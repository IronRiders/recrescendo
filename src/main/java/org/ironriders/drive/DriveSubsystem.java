package org.ironriders.drive;

import java.io.IOException;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import org.ironriders.lib.Constants;
import org.ironriders.lib.IronSubsystem;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.controller.ProfiledPIDController;
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
    private final DriveCommands commands;

    private static SwerveDrive swerveDrive;
    private static boolean rotationInvert = false;
    private static boolean driveInvert = false;

    public static boolean PIDRotation = false;
    public static boolean PIDPosition = false;

    public static AtomicBoolean isDriving = new AtomicBoolean(false);

    public static Pigeon2 pigeon = new Pigeon2(11);

    private static ProfiledPIDController rotationPid = new ProfiledPIDController(Constants.Drive.ROTATE_TO_TARGET_P,
            Constants.Drive.ROTATE_TO_TARGET_I,
            Constants.Drive.ROTATE_TO_TARGET_D, Constants.Drive.ROTATION_CONSTRAINTS);

    private static ProfiledPIDController xPid = new ProfiledPIDController(Constants.Drive.POSITION_P,
            Constants.Drive.POSITION_I, Constants.Drive.POSITION_D, Constants.Drive.TRANSLATION_CONSTRAINTS);
    private static ProfiledPIDController yPid = new ProfiledPIDController(Constants.Drive.POSITION_P,
            Constants.Drive.POSITION_I, Constants.Drive.POSITION_D, Constants.Drive.TRANSLATION_CONSTRAINTS);

    public DriveSubsystem() throws RuntimeException {
        try {
            swerveDrive = new SwerveParser(Constants.Drive.SWERVE_JSON_DIRECTORY) // YAGSL reads from the deploy/swerve
                    // directory.
                    .createSwerveDrive(Constants.Drive.SWERVE_MAX_TRANSLATION_TELEOP);
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
                (speeds, feedforwards) -> {
                    System.out.println("PathPlanner calling drive: vx=" + speeds.vxMetersPerSecond +
                            " vy=" + speeds.vyMetersPerSecond +
                            " omega=" + speeds.omegaRadiansPerSecond);
                    swerveDrive.drive(speeds);
                },
                Constants.Drive.HOLONOMIC_CONFIG,
                robotConfig,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);

        rotationPid.reset(getRotation());
        rotationPid.enableContinuousInput(0, Math.PI * 2);
        rotationPid.setTolerance(0.05);

        xPid.reset(getPose().getX());
        yPid.reset(getPose().getY());

        Pathfinding.setPathfinder(new LocalADStar());
    }

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();

        if (!isDriving.get() && (PIDRotation || PIDPosition)) {
            drive(new Translation2d(), 0, true);
        }

        publish("x PID", xPid);
        publish("y PID", yPid);
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
        isDriving.getAndSet(true);

        if (PIDRotation && PIDPosition) {
            swerveDrive.drive(getNextPose(),
                    -rotationPid.calculate(getRotation()),
                    fieldRelative,
                    false);
        } else if (PIDRotation && !PIDPosition) {
            swerveDrive.drive(translation.times(driveInvert ? -1 : 1),
                    -rotationPid.calculate(getRotation()),
                    fieldRelative,
                    false);
        } else if (!PIDRotation && PIDPosition) {
            swerveDrive.drive(getNextPose(),
                    rotation * (rotationInvert ? -1 : 1),
                    fieldRelative,
                    false);
        } else {
            swerveDrive.drive(
                    translation.times(driveInvert ? -1 : 1),
                    rotation * (rotationInvert ? -1 : 1),
                    fieldRelative,
                    false);
        }

        isDriving.getAndSet(false);
    }

    /**
     * @return The robot's current rotation.
     */
    public static double getRotation() {
        return getPose().getRotation().getRadians();
    }

    /** Where is the robot? */
    public static Pose2d getPose() {
        return DriveSubsystem.swerveDrive.getPose();
    }

    /*
     * Enable and disable PID rotation control. Set goal using {@link
     * #setRotationGoal()}
     */
    public static void setPIDRotationControl(boolean PIDControl) {
        PIDRotation = PIDControl;

        if (!PIDControl) {
            rotationPid.reset(getRotation());
        }
    }

    /*
     * Enable and disable PID position control.
     */
    public static void setPIDPositionControl(boolean PIDControl) {
        PIDPosition = PIDControl;

        if (!PIDControl) {
            xPid.reset(getPose().getX());
            yPid.reset(getPose().getY());
        }
    }

    /**
     * Sets the PID rotation goal in degrees.
     */
    public static void setRotationGoal(double goal) {
        rotationPid.setGoal(Math.toRadians(goal));
    }

    /**
     * Sets the PID rotation goal in radians.
     */
    public static void setRotationGoalRad(double goal) {
        rotationPid.setGoal(goal);
    }

    public static void setPositionGoal(Translation2d target) {
        xPid.setGoal(target.getX());
        yPid.setGoal(target.getY());
    }

    public static Translation2d getNextPose() {
        return new Translation2d(xPid.calculate(getPose().getX()), yPid.calculate(getPose().getY()));
    }

    public static boolean atGoal() {
        return xPid.atGoal() && yPid.atGoal();
    }

    public static Command pathfindToPose(Pose2d target) {
        return new Command() {
            PathPlannerPath path;
            List<Waypoint> waypoints;
            int i = 0;

            @Override
            public void initialize() {
                Pathfinding.ensureInitialized();
                Pathfinding.setStartPosition(getPose().getTranslation());
                Pathfinding.setGoalPosition(target.getTranslation());

                path = Pathfinding.getCurrentPath(Constants.Drive.PATHFIND_CONSTRAINTS,
                        new GoalEndState(0, target.getRotation()));

                waypoints = path.getWaypoints();
                i = 0;

                if (!waypoints.isEmpty()) {
                    setPositionGoal(waypoints.get(i).nextControl());
                }
            }

            @Override
            public void execute() {
                if (atGoal() && i < waypoints.size() - 1) {
                    i++;
                    setPositionGoal(waypoints.get(i).nextControl());
                }
            }

            @Override
            public boolean isFinished() {
                return i >= waypoints.size() - 1 && atGoal();
            }
        };
    }

    /** Fetch the DriveCommands instance */
    public DriveCommands getCommands() {
        return commands;
    }

    /** Fetch the SwerveDrive instance */
    public static SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    /**
     * Sets the maximum translation speed for the swerve drive.
     * 
     * @param max The maximum translation speed in meters per second.
     */
    public static void setSpeedMax(double max) {
        swerveDrive.setMaximumAllowableSpeeds(max, Constants.Drive.SWERVE_MAX_ANGULAR_TELEOP);
    }

    /**
     * Opens a {@link Pidgeon2} sensor and gets yaw, waits 1 second or until the
     * signal updates, then gets that
     * value as double.
     */
    public static void resetRotation() {
        pigeon.reset();
        resetOdometry(swerveDrive.getPose());
        rotationPid.reset(0);
    }

    /**
     * Sets the robot's odometry to a given pose with rotation at 0.
     * 
     * @param pose2d The pose to reset the odometry to.
     */
    public static void resetOdometry(Pose2d pose2d) {
        swerveDrive.resetOdometry(new Pose2d(pose2d.getTranslation(), new Rotation2d(0)));
    }

    /**
     * Inverts the rotation controls.
     */
    public void switchRotation() {
        rotationInvert = !rotationInvert;
    }

    /**
     * Inverts the drive controls.
     */
    public void switchDrive() {
        driveInvert = !driveInvert;
    }
}