package org.ironriders.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import java.io.IOException;
import org.ironriders.lib.Constants.Drive;
import org.ironriders.lib.IronSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * The DriveSubsystem encompasses everything that the Swerve Drive needs to function. It keeps track
 * of the robot's position and angle, and uses the controller input to figure out how the individual
 * modules need to turn and be angled.
 */
public class DriveSubsystem extends IronSubsystem {

  private final DriveCommands commands;

  private SwerveDrive swerveDrive;
  private boolean rotationInvert = false;
  private boolean driveInvert = false;

  public Command pathfindCommand;
  public double controlSpeedMultipler = 1;

  private boolean enableVision = false;
  private PhotonCamera camera = new PhotonCamera(Drive.VISION_CAMERA);
  private PIDController visPidController =
      new PIDController(Drive.VISION_P, Drive.VISION_I, Drive.VISION_D);
  private double distance = 0;

  public DriveSubsystem() throws RuntimeException {
    try {
      swerveDrive =
          new SwerveParser(Drive.SWERVE_JSON_DIRECTORY) // YAGSL reads from the deploy/swerve
              // directory.
              .createSwerveDrive(Drive.SWERVE_MAX_TRANSLATION_TELEOP);
    } catch (
        IOException e) { // instancing SwerveDrive can throw an error, so we need to catch that.
      throw new RuntimeException("Error configuring swerve drive", e);
    }

    commands = new DriveCommands(this);

    swerveDrive.setHeadingCorrection(false);
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    RobotConfig robotConfig = null;
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      throw new RuntimeException("Could not load path planner config", e);
    }

    AutoBuilder.configure(
        swerveDrive::getPose,
        swerveDrive::resetOdometry,
        swerveDrive::getRobotVelocity,
        (speeds, feedforwards) -> swerveDrive.setChassisSpeeds(speeds),
        Drive.HOLONOMIC_CONFIG,
        robotConfig,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
    visionInit();
  }

  /**
   * Vrrrrooooooooom brrrrrrrrr BRRRRRR wheeee BRRR brrrr VRRRRROOOOOOM ZOOOOOOM ZOOOOM WAHOOOOOOOOO
   * WAHAHAHHA (Drives given a desired translation and rotation.)
   *
   * @param translation Desired translation in meters per second.
   * @param rotation Desired rotation in radians per second.
   * @param fieldRelative If not field relative, the robot will move relative to its own rotation.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    // I'm sure this will be problem at some point sorry
    if (!enableVision) {
      swerveDrive.drive(
          translation.times(driveInvert ? -1 : 1),
          rotation * (rotationInvert ? -1 : 1),
          fieldRelative,
          false);
    }
  }

  /** Fetch the DriveCommands instance */
  public DriveCommands getCommands() {
    return commands;
  }

  /** Fetch the SwerveDrive instance */
  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }
  /** Where is the robot? */
  public Pose2d getPose() {
    return this.swerveDrive.getPose();
  }

  @Override
  public void periodic() {
    visionPeriodic(enableVision);
  }

  /**
   * Vision Main loop
   *
   * @param controlsDrive Am I allowed to move?
   */
  private void visionPeriodic(boolean controlsDrive) {
    boolean targetVisible = false;
    double targetYaw = 0.0;
    var results = camera.getAllUnreadResults();

	visPidController.setSetpoint(0);

    if (!results.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        // At least one AprilTag was seen by the camera
        for (var target : result.getTargets()) {
          if (target.getFiducialId() == 7) {
            // Found Tag 7, record its information
            targetYaw = target.getYaw();
			//We assume the camera and tag are both at a meter of height, but this is a very bad idea as the differance is important. Real nums tbd
            distance = PhotonUtils.calculateDistanceToTargetMeters(1, 1, 0, target.getPitch());
            targetVisible = true;
          }
        }
      }
    }

    publish("Camera sees target", targetVisible);
    publish("Distance to target", distance);
	publish("Vision can drive", controlsDrive);

    if (targetVisible) {
      // We found our favorite toy! (tag #7)
      publish("Yaw offset", targetYaw);
      double requestedmovement = visPidController.calculate(targetYaw);
      publish("Requested movement", requestedmovement);

      if (controlsDrive) {

          if (requestedmovement > Drive.VISION_ROTATION_MAX_SPEED) {
            requestedmovement = Drive.VISION_ROTATION_MAX_SPEED;
          }
		  if (requestedmovement < -Drive.VISION_ROTATION_MAX_SPEED) {
            requestedmovement = -Drive.VISION_ROTATION_MAX_SPEED;
          }
        
        swerveDrive.drive(new Translation2d(0, 0), requestedmovement * -1, false, true);
      }
    } else {

      if (controlsDrive) {
        // Saftey measure, if vision control is requested but we lose the tag, stop moving.
        // Otherwise we will just keep moving the preiviously commanded direction forever
        swerveDrive.drive(new Translation2d(0, 0), 0, false, true);
      }
    }
  }

  /** Initalize vision system. Disables anyone elses control*/
  private void visionInit() {
    publish("Camera sees target", 0);
    publish("Requested movement", 0);
    publish("Distance to target", 0);
  }

  /** Set if vision is allowed to drive. */
  public void setVisionControl(boolean state) {
    this.enableVision = state;
  }

  public void resetRotation() {
    Pigeon2 pigeon2 = new Pigeon2(9);
    swerveDrive.resetOdometry(
        new Pose2d(
            swerveDrive.getPose().getTranslation(),
            new Rotation2d(
                pigeon2.getYaw(true).waitForUpdate(1).getValueAsDouble() * (Math.PI / 180f))));
    pigeon2.close();
  }

  public void resetOdometry(Pose2d pose2d) {
    swerveDrive.resetOdometry(new Pose2d(pose2d.getTranslation(), new Rotation2d(0)));
  }

  public void switchRotation() {
    rotationInvert = !rotationInvert;
  }

  public void switchDrive() {
    driveInvert = !driveInvert;
  }

  public void setSpeed(double speed) {
    controlSpeedMultipler = speed;
  }
}
