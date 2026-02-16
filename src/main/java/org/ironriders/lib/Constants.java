package org.ironriders.lib;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class Constants {
    public class Robot {

        public static final double COMPENSATED_VOLTAGE = 10.0;
    }

    public class Identifiers {

        public static final int CONTROLLER_PRIMARY_PORT = 0;
        public static final int CONTROLLER_SECONDARY_PORT = 1;

        public static final int LAUNCHER_MOTOR_RIGHT = 9;
        public static final int LAUNCHER_MOTOR_LEFT = 10;

        public static final int PIVOT_MOTOR = 12;
        public static final int PIVOT_ENCODER = 0;

        public static final int INTAKE_MOTOR = 13;

        public static final int CLIMBER_MOTOR_RIGHT = 14;
        public static final int CLIMBER_MOTOR_LEFT = 15;

        public static final int LIGHTING_STRIP_PORT = 5;
    }

    public class Drive {
        public static final File SWERVE_JSON_DIRECTORY = new File(Filesystem.getDeployDirectory(), "swerve");

        public static final PPHolonomicDriveController HOLONOMIC_CONFIG = new PPHolonomicDriveController(
                new PIDConstants(10.0, 0.0, 0.0), // Translation PID
                new PIDConstants(10.0, 0.0, 0.0) // Rotation PID
        );

        public static final double ROTATE_TO_TARGET_P = 8;
        public static final double ROTATE_TO_TARGET_I = 0;
        public static final double ROTATE_TO_TARGET_D = 0;

        public static final double POSITION_P = 6;
        public static final double POSITION_I = 0;
        public static final double POSITION_D = 0;

        public static final Constraints ROTATION_CONSTRAINTS = new Constraints(Math.PI, Math.PI / 1.2); // Radians

        public static final double TRANSLATION_CONTROL_EXPONENT = 3.0;
        public static final double TRANSLATION_CONTROL_DEADBAND = 0.8;
        public static final double ROTATION_CONTROL_EXPONENT = 3.0;
        public static final double ROTATION_CONTROL_DEADBAND = 0.8;

        public static final double SWERVE_MAX_TRANSLATION_TELEOP = 4; // m/s
        public static final double SWERVE_MAX_ANGULAR_TELEOP = Math.PI / 1.2; // rad/s

        public static final double SWERVE_MAX_TRANSLATION_PATHFIND = 4; // m/s
        public static final double SWERVE_MAX_ANGULAR_PATHFIND = Math.PI / 1.2; // rad/s

        public static final double SWERVE_MAX_TRANSLATION_ACCEL_PATHFIND = SWERVE_MAX_TRANSLATION_PATHFIND / 1.2;
        public static final double SWERVE_MAX_ANGULAR_ACCEL_PATHFIND = SWERVE_MAX_ANGULAR_PATHFIND / 1.2;

        public static final Constraints TRANSLATION_CONSTRAINTS = new Constraints(SWERVE_MAX_TRANSLATION_PATHFIND,
                SWERVE_MAX_TRANSLATION_ACCEL_PATHFIND);

        public static final PathConstraints PATHFIND_CONSTRAINTS = new PathConstraints(SWERVE_MAX_TRANSLATION_PATHFIND,
                SWERVE_MAX_TRANSLATION_ACCEL_PATHFIND, SWERVE_MAX_ANGULAR_PATHFIND, SWERVE_MAX_ANGULAR_ACCEL_PATHFIND);

        public static final double DRIVE_OVERRIDE_THRESHOLD = 0.3;
    }

    public class Vision {
        public static final List<VisionCamera> CAMERAS = new ArrayList<VisionCamera>();

        static {
            CAMERAS.add(new VisionCamera("main", new Transform3d(
                    new Translation3d(
                            -0.25, // forward (meters)
                            0.0, // left (meters)
                            0.5 // up (meters)
                    ),
                    new Rotation3d(
                            0.0, // roll
                            0.0, // pitch
                            Math.PI // yaw
                    ))));
        }

        public static final Double SKEW_THROWAWAY_THRESHOLD = 15d; // deg,
        public static final Double POSE_DISTANCE_THROWAWAY_THRESHOLD = 6d; // meters, TODO: Tune
        public static final Double TARGET_DISTANCE_THROWAWAY_THRESHOLD = 5d; // meters, TODO: Tune

        public static final Double WEIGHT_SCALE = 5d;

        public static class VisionCamera {
            String m_name;
            Transform3d m_offset;
            Double m_trustWeight;
            
            PhotonCamera m_photonCamera;
            PhotonPoseEstimator m_estimator;

            AprilTagFieldLayout m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

            public PhotonCamera getPhotonCamera() {
                return m_photonCamera;
            }

            public PhotonPoseEstimator getEstimator() {
                return m_estimator;
            }

            public String getName() {
                return m_name;
            }

            public Double getWeight() {
                return m_trustWeight;
            }

            /*
             * Define a new camera.
             * 
             * @param name is the camera name set in the photonvision dashboard.
             * 
             * @param offset is the offset from the center of the robot, positive x towards
             * the battery.
             * 
             * @param trustWeight is the weight on the trust we have in estimations made
             * by this camera. Useful if you have one crapy camera and one good one or
             * something similar. Should be in the range [-1 (least trusting) to 1 (most
             * trusting)]. Zero is no weighing.
             */
            public VisionCamera(String name, Transform3d offset, Double trustWeight) {
                m_name = name;
                m_offset = offset;
                m_trustWeight = Utils.clamp(-1, 1, trustWeight);

                m_photonCamera = new PhotonCamera(name);
                m_estimator = new PhotonPoseEstimator(m_fieldLayout, offset);
            }

            /*
             * Define a new camera.
             * 
             * @param name is the camera name set in the photonvision dashboard.
             * 
             * @param offset is the offset from the center of the robot, positive x towards
             * the battery.
             */
            public VisionCamera(String name, Transform3d offset) {
                m_name = name;
                m_offset = offset;
                m_trustWeight = 0d;

                m_photonCamera = new PhotonCamera(name);
                m_estimator = new PhotonPoseEstimator(m_fieldLayout, offset);
            }
        }
    }

    public class Intake {

        public static final int INTAKE_MOTOR_STALL_LIMIT = 20;
        public static final double EJECT_WAIT_TIME = 1;
        public static final double CENTER_TIMEOUT = 1;

        public enum State {
            INTAKE(-0.5),
            STOP(0),
            BACK(1),
            CENTER(-0.3);

            public double speed;

            private State(double speed) {
                this.speed = speed;
            }
        }
    }

    public class Launcher {

        public static final int LAUNCHER_MOTOR_STALL_LIMIT = 30;
        public static final double LAUNCH_TIMEOUT = 0.4;

        public static final double CONTROL_P = 0.00017;
        public static final double CONTROL_I = 0.0006;
        public static final double CONTROL_D = 0.0;

        public static final double CONTROL_SPEED_TOLERANCE = 2000; // in RPM

        public enum State { // in RPM
            LAUNCH(3500),
            STOP(0);

            public double speed;

            private State(double speed) {
                this.speed = speed;
            }
        }
    }

    public class Lights {

        public static final int STRIP_LENGTH = 28;

        public static final Distance STRIP_DENSITY = Meters.of(1 / 120.0);

        public enum ColorState {
            GREEN(0, 255, 0),
            WHITE(255, 255, 255);

            public int r;
            public int g;
            public int b;

            ColorState(int r, int g, int b) {
                this.r = r;
                this.g = g;
                this.b = b;
            }
        }

        public enum State {
            OFF(1),
            RGB(1),
            TRANS(0.5),
            GAY(0.5),
            NONBINARY(0.5);

            public double scrollSpeed;

            State(double scrollSpeed) {
                this.scrollSpeed = scrollSpeed;
            }
        }
    }

    public class Pivot {

        public static final int MOTOR_CURRENT_LIMIT = 20;

        public static final double CONTROL_P = 0.005;
        public static final double CONTROL_I = 0.0;
        public static final double CONTROL_D = 0.0;

        public static final double CONTROL_T = 0.2;

        public static final double CONTROL_TOLERANCE = 5;

        public static final double GEAR_RATIO = 1.0; // 1/(27*2);

        public static final TrapezoidProfile.Constraints CONTROL_CONSTRAINTS = new TrapezoidProfile.Constraints(20, 10);

        public static final double ENCODER_OFFSET = 260;

        public enum State {
            GROUND(42),
            STOWED(173),
            LAUNCHER(256);

            public final double position;

            private State(int position) {
                this.position = position;
            }
        }
    }
}
