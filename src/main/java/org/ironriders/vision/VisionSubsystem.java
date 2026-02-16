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
import org.ironriders.lib.Constants.Vision.VisionCamera;
import org.ironriders.lib.IronSubsystem;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;

public class VisionSubsystem extends IronSubsystem {
    public enum TagInvalidReason {
        NO_SKEW,
        TOO_DISTANT,
        TOO_CLOSE
    }

    private final VisionCommands commands = new VisionCommands(this);

    private String debugString;

    public static AprilTagFieldLayout fieldLayout;

    public VisionSubsystem() {
        try {
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        } catch (UncheckedIOException e) {
            reportError("Could not load apriltag layout!");
            e.printStackTrace();
        }
    }

    @Override
    public void periodic() {
        // for every camera...
        Vision.CAMERAS.parallelStream().forEach((camera) -> {
            camera.updateResultBuffer();

            if (!camera.getResult().hasTargets()) {
                return; // We don't see any tags, give up.
            }

            // estimate the pose
            estimateRobotPose(camera);
        });
    }

    /*
     * Try to estimate how much we should trust the opinion of this camera. Higher
     * numbers mean less trust.
     */
    public Vector<N3> estimateStdDevVector(List<PhotonTrackedTarget> targets, VisionCamera camera) {
        double xyStdDev;
        double thetaStdDev;

        double avgDistance = targets.stream()
                .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
                .average()
                .orElse(-1);

        // TODO: These numbers are mostly arbitrary.
        if (targets.size() > 1) { // Multi target
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

        // add camera weighting. Inverted because higher numbers are less trusting.
        xyStdDev += -camera.getWeight() * Vision.WEIGHT_SCALE;
        thetaStdDev += -camera.getWeight() * Vision.WEIGHT_SCALE;

        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }

    /*
     * Try to estimate the position of the robot using the visible tags.
     * This function should be called for every camera.
     */
    public void estimateRobotPose(VisionCamera camera) {
        List<PhotonTrackedTarget> validTargets = new ArrayList<PhotonTrackedTarget>();
        Map<PhotonTrackedTarget, String> tagStrings = new HashMap<PhotonTrackedTarget, String>();

        PhotonPipelineResult result = camera.getResult();

        // for every target (tag)...
        for (PhotonTrackedTarget target : result.getTargets()) {
            makeDebugString(target);

            // get the skew (the angle off of straight on)
            Double skew = calculateSkew(target);

            // get the distance from the camera to the target.
            double distance = target.getBestCameraToTarget().getTranslation().getNorm();

            String distanceString = String.format("%03.2f", distance);

            // -- Checks to make sure that tag is valid --

            // if we are too normal to the tag, we can't trust the result. this is for
            // complicated reasons involving how photon vision sees tags. ask Issy in
            // discord if you really need to know (you probably don't)
            if (Math.abs(skew) < Constants.Vision.SKEW_THROWAWAY_THRESHOLD) {
                reportWarning("No skew");
                addBadTagToString(TagInvalidReason.NO_SKEW, String.valueOf(skew));

                continue;
            }

            // the distance is negative, something has gone wrong.
            if (distance < 0) {
                reportWarning("Target too close");
                addBadTagToString(TagInvalidReason.TOO_CLOSE, distanceString);

                continue;
            }

            // if the distance is too great, we can't trust that the tag will be read
            // reliably, so just ignore it.
            if (distance > Constants.Vision.TARGET_DISTANCE_THROWAWAY_THRESHOLD) {
                reportWarning("Target too distant");
                addBadTagToString(TagInvalidReason.TOO_DISTANT, distanceString);

                continue;
            }

            addGoodTagToString(distanceString);
            tagStrings.put(target, debugString);

            // tag is valid! yay :3
            validTargets.add(target);
        }

        List<PhotonTrackedTarget> invalidTargets = result.targets;
        invalidTargets.removeAll(validTargets);

        publish("Invalid targets",
                invalidTargets.stream().map(t -> String.valueOf(t.fiducialId)).collect(Collectors.joining(" | ")));

        // set the targets in the pipeline result to only be the valid ones. (kinda
        // silly but better than constructing a new pipeline result)
        result.targets = validTargets;

        publish("Valid targets:",
                validTargets.stream().map(PhotonTrackedTarget::getFiducialId).map(i -> String.valueOf(i))
                        .collect(Collectors.joining(" | ")));

        publish(String.format("Tag data for camera: %s", camera.getName()), tagStrings.values().stream().sorted().collect(Collectors.joining(" | ")));

        EstimatedRobotPose estimatedPose;

        if (result.getTargets().size() > 1) {
            // if we have more than one tag, do multi-tag estimation,
            estimatedPose = camera.getEstimator().estimateCoprocMultiTagPose(result).orElse(null);
        } else if (result.getTargets().size() == 1) {
            // if we only have one, do single tag.
            estimatedPose = camera.getEstimator().estimateLowestAmbiguityPose(result).orElse(null);
        } else {
            // otherwise, we must not have any, show a warning and exit.
            reportWarning("No valid targets");
            return;
        }

        if (estimatedPose == null) {
            // Something has gone wrong, give up and try again next tick.
            reportWarning("Estimated pose was null");
            return;
        }

        // Throw away the new pose if it is too far away.
        if (estimatedPose.estimatedPose.getTranslation()
                .getDistance(DriveSubsystem.getPose3d().getTranslation()) > Vision.POSE_DISTANCE_THROWAWAY_THRESHOLD) {
            reportWarning("Estimated pose too distant");
            return;
        }

        // Actually add the estimate. Adding an estimate for each camera is apparently
        // the right way to do it, but seems wrong. idk
        DriveSubsystem.getSwerveDrive().setVisionMeasurementStdDevs(estimateStdDevVector(validTargets, camera));
        DriveSubsystem.getSwerveDrive().addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(),
                Timer.getFPGATimestamp());
    }

    // debugging helper functions
    public void makeDebugString(PhotonTrackedTarget target) {
        debugString = "Tag " + String.valueOf(target.getFiducialId()) + ": ";
    }

    public void addBadTagToString(TagInvalidReason reason, String extra) {
        if (debugString != null) {
            debugString += "BAD: " + reason.toString();
            if (extra != null) {
                debugString += " | " + extra;
            }
        }
    }

    public void addGoodTagToString(String extra) {
        if (debugString != null) {
            debugString += "GOOD: ";
            if (extra != null) {
                debugString += " | " + extra;
            }
        }
    }

    /*
     * Calculate the skew for a tag
     */
    public double calculateSkew(PhotonTrackedTarget target) {
        return (target.getBestCameraToTarget().getRotation().getZ() * 180.0 / Math.PI) - 90;
    }

    /*
     * Get a list of the angles of a tag
     */
    public Double[] getTargetAngles(PhotonTrackedTarget target) {
        return new Double[] { target.getYaw(), target.getPitch(), calculateSkew(target) };
    }

    public VisionCommands getCommands() {
        return commands;
    }
}
