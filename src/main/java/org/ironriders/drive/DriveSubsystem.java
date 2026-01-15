package org.ironriders.drive;

import java.io.IOException;

import org.ironriders.lib.Constants.Drive;
import org.ironriders.lib.Constants.Drive.Controller;
import org.ironriders.lib.IronSubsystem;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * The DriveSubsystem encompasses everything that the Swerve Drive needs to
 * function. It keeps track
 * of the robot's position and angle, and uses the controller input to figure
 * out how the individual
 * modules need to turn and be angled.
 */
public class DriveSubsystem extends IronSubsystem {
  public static Controller controller;
  private final DriveCommands commands;

  private static SwerveDrive swerveDrive;
  private static boolean rotationInvert = false;
  private static boolean driveInvert = false;

  public DriveSubsystem() throws RuntimeException {
    try {
      swerveDrive = new SwerveParser(Drive.SWERVE_JSON_DIRECTORY) // YAGSL reads from the deploy/swerve
          // directory.
          .createSwerveDrive(Drive.SWERVE_MAX_TRANSLATION_TELEOP);
    } catch (IOException e) { // instancing SwerveDrive can throw an error, so we need to catch that.
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
  }

  /**
   * Vrrrrooooooooom brrrrrrrrr BRRRRRR wheeee BRRR brrrr VRRRRROOOOOOM ZOOOOOOM
   * ZOOOOM WAHOOOOOOOOO
   * WAHAHAHHA (Drives given a desired translation and rotation.)
   *
   * @param translation   Desired translation in meters per second.
   * @param rotation      Desired rotation in radians per second.
   * @param fieldRelative If not field relative, the robot will move relative to
   *                      its own rotation.
   */
  public static void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(
        translation.times(driveInvert ? -1 : 1),
        rotation * (rotationInvert ? -1 : 1),
        fieldRelative,
        false);
  }

  public static int requestDriveMovement(Controller requester, Translation2d translation, double rotation,
      boolean fieldRelative) {
    if (controller == requester) {
      drive(translation, rotation, fieldRelative);
      return 0; // Succeeded.
    } else {
      return 1; // Failed.
    }
  }

  public static int requestDriveStop(Controller requester) {
    return requestDriveMovement(requester, new Translation2d(0, 0), 0, false);
  }

  public static Command pathfindToPose(Pose2d target, PathConstraints constraints) {
    Command pathfindCommand = AutoBuilder.pathfindToPose(target, constraints);
    return pathfindCommand.withName("Pathfind to " + target.getX() + ", " + target.getY());
  }

  public static Command pathfindToPose(Pose2d target) {
    Command pathfindCommand = AutoBuilder.pathfindToPose(target, Drive.PATHFIND_CONSTRAINTS);
    return pathfindCommand.withName("Pathfind to " + target.getX() + ", " + target.getY());
  }

  public static Command pathfindThenFollowPath(PathPlannerPath path, PathConstraints constraints) {
    Command pathfindCommand = AutoBuilder.pathfindThenFollowPath(
        path,
        constraints);

    return pathfindCommand.withName("Pathfind to " + path.name);
  }

  public static Command pathfindThenFollowPath(PathPlannerPath path) {
    Command pathfindCommand = AutoBuilder.pathfindThenFollowPath(
        path, Drive.PATHFIND_CONSTRAINTS);

    return pathfindCommand.withName("Pathfind to " + path.name);
  }

  public static void setController(Controller target) {
    controller = target;
  }

  /** Fetch the DriveCommands instance */
  public DriveCommands getCommands() {
    return commands;
  }

  /** Fetch the SwerveDrive instance */
  public static SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  /** Where is the robot? */
  public Pose2d getPose() {
    return DriveSubsystem.swerveDrive.getPose();
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
}
