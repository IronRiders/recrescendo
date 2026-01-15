package org.ironriders.drive;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

import org.ironriders.lib.Constants.Drive;
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

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;

public class VisionSubsystem extends IronSubsystem {
    private PhotonCamera camera = new PhotonCamera(Vision.VISION_CAMERA);
    private PIDController visPidController = new PIDController(Vision.VISION_P, Vision.VISION_I, Vision.VISION_D);
    private final PhotonPoseEstimator poseEstimator;

    private double distance = 0;
    private boolean targetVisible = false;
    private List<PhotonTrackedTarget> targets;
    private List<PhotonPipelineResult> results;
    private PhotonPipelineResult result;
    private AprilTagFieldLayout fieldLayout;

    public VisionSubsystem() {
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource("deploy/2026-rebuilt-welded.json"); /*
                                                                                                    * TODO: When WPI
                                                                                                    * gets
                                                                                                    * around to adding
                                                                                                    * the rebuilt tags
                                                                                                    * change this
                                                                                                    * to use the proper
                                                                                                    * one.
                                                                                                    */
        } catch (Exception e) {
            DriverStation.reportError("Failed to load AprilTag layout", e.getStackTrace());
            throw new RuntimeException(e);
        }

        poseEstimator = new PhotonPoseEstimator(
                fieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                Vision.CAMERA_OFFSET);

        poseEstimator.setMultiTagFallbackStrategy( // What to do if we can only see one tag.
                PoseStrategy.LOWEST_AMBIGUITY);
    }

    public void estimateRobotPose() {
        EstimatedRobotPose newPose = poseEstimator.update(result).orElse(null); // Uses a deprecated method but idk how
                                                                                // else to do it.
        if (newPose == null) {
            // Something has gone wrong, give up and try again next tick.
            return;
        }
        // Actually add the estimate
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

    public List<Double> getTargetAngles(PhotonTrackedTarget target) {
        List<Double> targetRotation;
        targetRotation.add(target.getYaw());
        targetRotation.add(target.getPitch());
        targetRotation.add(target.getSkew());

        return targetRotation;
    }

    @Override
    public void periodic() {
        visPidController.setSetpoint(0); // Not sure why we do this.

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
        publish("Sees target", result.hasTargets());

        Map<PhotonTrackedTarget, Double> m = new HashMap<>();

        for (var target : result.getTargets()) {
            m.put(target, getDistance(target));

            switch (target.getFiducialId()) {
                case 7:
                    // We found our favorite toy! (tag #7)
                    double requestedMovement = Utils.clamp(-Vision.VISION_ROTATION_MAX_SPEED, 
                        Vision.VISION_ROTATION_MAX_SPEED, 
                        visPidController.calculate(target.getYaw()));

                    publish("Yaw, Pitch, Skew", getTargetAngles(target).toString());
                    publish("Requested movement", requestedMovement);

                    DriveSubsystem.setController(Controller.VISION);
                    DriveSubsystem.requestDriveMovement(Controller.VISION, new Translation2d(0, 0), -requestedMovement, false);
                    break;

                default:
                    break;
            }
        }
        publish("Tags -> Distances", m.toString());
    }
}
