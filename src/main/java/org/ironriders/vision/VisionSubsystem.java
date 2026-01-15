package org.ironriders.vision;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ironriders.drive.DriveSubsystem;
import org.ironriders.lib.Constants.Drive.Controller;
import org.ironriders.lib.Constants.Vision;
import org.ironriders.lib.IronSubsystem;
import org.ironriders.lib.Utils;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Filesystem;

public class VisionSubsystem extends IronSubsystem {
    private final VisionCommands commands = new VisionCommands(this);

    private PhotonCamera camera = new PhotonCamera(Vision.VISION_CAMERA);
    private PIDController visPidController = new PIDController(Vision.VISION_P, Vision.VISION_I, Vision.VISION_D);
    private final PhotonPoseEstimator poseEstimator;

    private List<PhotonTrackedTarget> targets;
    private List<PhotonPipelineResult> results;
    private PhotonPipelineResult result;
    
    public static AprilTagFieldLayout fieldLayout;

    public VisionSubsystem() {
        try {
            // TODO: When WPI gets around to adding the rebuilt tags change this to use the
            // proper one.
            Path layoutPath = Filesystem.getDeployDirectory()
                    .toPath()
                    .resolve("2026-rebuilt-welded.json");

            fieldLayout = new AprilTagFieldLayout(layoutPath);
        } catch (IOException e) {
            reportError("Could not load apriltag layout!");
            e.printStackTrace();
        }

        poseEstimator = new PhotonPoseEstimator(
                fieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                Vision.CAMERA_OFFSET);

        poseEstimator.setMultiTagFallbackStrategy( // What to do if we can only see one tag.
                PoseStrategy.LOWEST_AMBIGUITY);
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
            xyStdDev = 0.05 + (avgDistance * 0.02);
            thetaStdDev = Math.toRadians(2 + avgDistance);
        } else { // Single Target
            xyStdDev = 0.5 + (avgDistance * 0.1);
            thetaStdDev = Math.toRadians(10 + avgDistance * 5); // Really don't trust single tag rotation
        }

        double ambiguity = targets.stream()
                .mapToDouble(t -> t.getPoseAmbiguity())
                .average()
                .orElse(Double.POSITIVE_INFINITY) + 1d; // Make sure we really don't like this pose if the optional is null.

        // if we have high ambiguity, remove some trust.
        xyStdDev *= (ambiguity * 3.0);
        thetaStdDev *= (ambiguity * 5.0);

        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }

    public void estimateRobotPose() {
        EstimatedRobotPose newPose = poseEstimator.update(result).orElse(null); // Uses a deprecated method but idk how
                                                                                // else to do it.
        if (newPose == null) {
            // Something has gone wrong, give up and try again next tick.
            return;
        }
        // Actually add the estimate
        DriveSubsystem.getSwerveDrive().setVisionMeasurementStdDevs(estimateStdDevVector(newPose, targets));
        DriveSubsystem.getSwerveDrive().addVisionMeasurement(newPose.estimatedPose.toPose2d(),
                newPose.timestampSeconds);
    }

    public double getDistance(PhotonTrackedTarget target) {
        // *2, as the offset is from the center of the robot, and this wants the
        // distance
        // from the floor
        return PhotonUtils.calculateDistanceToTargetMeters(
                Vision.CAMERA_OFFSET.getZ() * 2,
                fieldLayout.getTagPose(target.getFiducialId())
                        .orElse(new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0))).getZ(),
                Vision.CAMERA_OFFSET.getRotation().getY(), // Pitch
                target.getPitch());
    }

    public Double[] getTargetAngles(PhotonTrackedTarget target) {
        return new Double[] { target.getYaw(), target.getPitch(), target.getSkew() };
    }

    @Override
    public void periodic() {
        results = camera.getAllUnreadResults();

        if (results.isEmpty()) {
            return; // Immediately give up if there is no new work to do.
        }

        PhotonPipelineResult result = results.get(results.size() - 1); // We only care about the most recent reading.

        if (!result.hasTargets()) {
            DriveSubsystem.requestDriveStop(Controller.VISION); // for testing, just stop if we don't see anything.
            return; // We don't see any tags, give up.
        }

        targets = result.getTargets();

        estimateRobotPose();

        // Testing code.
        visPidController.setSetpoint(0); // Assume we've rotated to face the target pose

        publish("Sees target?", result.hasTargets());

        Map<PhotonTrackedTarget, Double> m = new HashMap<>();

        for (var target : targets) {
            m.put(target, getDistance(target));

            switch (target.getFiducialId()) {
                case -1: // Error, not a valid tag!
                    reportWarning("Vision got an invalid tag!");
                    return;
                case 7:
                    // We found our favorite toy! (tag #7)
                    double requestedMovement = -Utils.clamp(-Vision.VISION_ROTATION_MAX_SPEED,
                            Vision.VISION_ROTATION_MAX_SPEED,
                            visPidController.calculate(target.getYaw()));

                    // Skew is horizontal offset from cam I think?
                    publish("Yaw, Pitch, Skew", getTargetAngles(target).toString());
                    publish("Requested movement", requestedMovement);

                    DriveSubsystem.requestDriveMovement(Controller.VISION, new Translation2d(0, 0), requestedMovement,
                            false);
                    break;

                default:
                    break;
            }
        }
        publish("Tags -> Distances", m.toString());
    }

    public VisionCommands getCommands() {
        return commands;
    }
}
