package org.ironriders.vision;

import java.io.UncheckedIOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import org.ironriders.drive.DriveSubsystem;
import org.ironriders.lib.Constants;
import org.ironriders.lib.Constants.Vision;
import org.ironriders.lib.IronSubsystem;
import org.ironriders.lib.Utils;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;

public class VisionSubsystem extends IronSubsystem {
    private final VisionCommands commands = new VisionCommands(this);

    private PhotonCamera camera = new PhotonCamera(Vision.VISION_CAMERA);
    private final PhotonPoseEstimator poseEstimator;

    private List<PhotonPipelineResult> results;

    private Double skew;

    public static AprilTagFieldLayout fieldLayout;

    public VisionSubsystem() {
        try {
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        } catch (UncheckedIOException e) {
            reportError("Could not load apriltag layout!");
            e.printStackTrace();
        }

        poseEstimator = new PhotonPoseEstimator(
                fieldLayout,
                Vision.CAMERA_OFFSET);
    }

    public Vector<N3> estimateStdDevVector(EstimatedRobotPose pose, List<PhotonTrackedTarget> targets) {
        double xyStdDev;
        double thetaStdDev;

        double avgDistance = pose.targetsUsed.stream()
                .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
                .average()
                .orElse(-1);

        // TODO: These numbers are mostly arbitrary.
        if (pose.targetsUsed.size() > 1) { // Multi target
            xyStdDev = 0.02 + (avgDistance * 0.03);
            thetaStdDev = Math.toRadians(1 + avgDistance);
        } else { // Single Target
            xyStdDev = 0.5 + (avgDistance * 0.1);
            thetaStdDev = Math.toRadians(10 + avgDistance * 5); // Really don't trust single tag rotation
        }

        double ambiguity = targets.stream()
                .mapToDouble(t -> t.getPoseAmbiguity())
                .average()
                .orElse(Double.POSITIVE_INFINITY) + 1d; // Make sure we really don't like this pose if the optional is
                                                        // null.

        // if we have high ambiguity, remove some trust.
        xyStdDev *= (ambiguity);
        thetaStdDev *= (ambiguity * 2.0);

        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }

    public void estimateRobotPose(PhotonPipelineResult result) {
        EstimatedRobotPose newPose;

        List<PhotonTrackedTarget> goodTargets = new ArrayList<PhotonTrackedTarget>();
        Map<PhotonTrackedTarget, String> tagStrings = new HashMap<PhotonTrackedTarget, String>();

        for (PhotonTrackedTarget target : result.getTargets()) {
            skew = calculateSkew(target);

            String str = "Tag " + String.valueOf(target.getFiducialId()) + ": ";

            if (Math.abs(skew) < Constants.Vision.SKEW_THROWAWAY_THRESHOLD) {
                reportWarning("Skew throwaway");
                str += "BAD: ";

                str += "not enough skew, " + String.valueOf(skew);

                tagStrings.put(target, str);
                continue;
            }

            double distance = target.getBestCameraToTarget().getTranslation().getNorm(); // TODO: fix
            String distString = String.format("%03.2f", distance);

            if (distance < 0) {
                reportWarning("Target inside us");
                str += "BAD: ";

                str += "Negative distance: " + distString;
                tagStrings.put(target, str);

                continue;
            }

            if (distance > Constants.Vision.TARGET_DISTANCE_THROWAWAY_THRESHOLD) {
                reportWarning("Target too distant");
                str += "BAD: ";
                str += "Too far: " + distString;
                tagStrings.put(target, str);

                continue;
            }
            str += "GOOD: ";

            str += "Distance: " + distString;
            tagStrings.put(target, str);

            goodTargets.add(target);
        }

        List<PhotonTrackedTarget> badTags = result.targets;
        badTags.removeAll(goodTargets);

        publish("Bad tags", badTags.stream().map(t -> String.valueOf(t.fiducialId)).collect(Collectors.joining(" | ")));

        result.targets = goodTargets;

        publish("Good tags:",
                goodTargets.stream().map(PhotonTrackedTarget::getFiducialId).map(i -> String.valueOf(i))
                        .collect(Collectors.joining(" | ")));

        publish("Tag data:", tagStrings.values().stream().sorted().collect(Collectors.joining(" | ")));

        if (result.getTargets().size() > 1) {
            newPose = poseEstimator.estimateCoprocMultiTagPose(result).orElse(null);
        } else {
            newPose = poseEstimator.estimateLowestAmbiguityPose(result).orElse(null);
        }

        if (newPose == null) {
            // Something has gone wrong, give up and try again next tick.
            reportWarning("Giving up in pose estimation!");
            return;
        }

        // Throwaway the pose if it is too normal to us or is too far away.
        if (Utils.getPoseDifference(Utils.flattenPose3d(newPose.estimatedPose),
                DriveSubsystem.getSwerveDrive().getPose()).getNorm() > Vision.POSE_DISTANCE_THROWAWAY_THRESHOLD) {
            reportWarning("new pose two distant");
            return;
        }

        // Actually add the estimate
        DriveSubsystem.getSwerveDrive().setVisionMeasurementStdDevs(estimateStdDevVector(newPose, goodTargets));
        DriveSubsystem.getSwerveDrive().addVisionMeasurement(newPose.estimatedPose.toPose2d(),
                Timer.getFPGATimestamp());
    }

    public double calculateSkew(PhotonTrackedTarget target) {
        return (target.getBestCameraToTarget().getRotation().getZ() * 180.0 / Math.PI) - 90;
    }

    /*
     * Get the distance to the provided target from the camera. Can throw an error
     * if the provided target's tag is not valid
     */
    public double getDistance(PhotonTrackedTarget target) {
        // *2, as the offset is from the center of the robot, and this wants the
        // distance from the floor
        return PhotonUtils.calculateDistanceToTargetMeters(
                Vision.CAMERA_OFFSET.getZ() * 2,
                fieldLayout.getTagPose(target.getFiducialId())
                        .orElseThrow().getZ(),
                Vision.CAMERA_OFFSET.getRotation().getY(), // Pitch
                target.getPitch());
    }

    public Double[] getTargetAngles(PhotonTrackedTarget target) {
        return new Double[] { target.getYaw(), target.getPitch(), calculateSkew(target) };
    }

    @Override
    public void periodic() {
        results = camera.getAllUnreadResults();

        if (results.isEmpty()) {
            return; // Immediately give up if there is no new work to do.
        }

        PhotonPipelineResult result = results.get(results.size() - 1); // We only care about the most recent reading.

        publish("Sees target?", result.hasTargets());

        if (result == null || !result.hasTargets()) {
            return; // We don't see any tags, give up.
        }

        estimateRobotPose(result);
    }

    public VisionCommands getCommands() {
        return commands;
    }
}
