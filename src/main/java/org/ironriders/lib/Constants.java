package org.ironriders.lib;
import static edu.wpi.first.units.Units.Meters;

import java.io.File;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Filesystem;


public class Constants {

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

        public static final int LIGHTING_STRIP_PORT = 0;
    }

    public class Drive {

        public static final File SWERVE_JSON_DIRECTORY = new File(Filesystem.getDeployDirectory(), "swerve");

        public static final PPHolonomicDriveController HOLONOMIC_CONFIG = new PPHolonomicDriveController(
                new PIDConstants(10.0, 0.05, 0.0), // Translation PID
                new PIDConstants(10.0, 0.2, 0.0) // Rotation PID
        );

        public static final double TRANSLATION_CONTROL_EXPONENT = 3.0;
        public static final double TRANSLATION_CONTROL_DEADBAND = 0.8;
        public static final double ROTATION_CONTROL_EXPONENT = 3.0;
        public static final double ROTATION_CONTROL_DEADBAND = 0.8;

        public static final double SWERVE_MAX_TRANSLATION_TELEOP = 4; // m/s
        public static final double SWERVE_MAX_ANGULAR_TELEOP = Math.PI * 3; // rad/s
    }

    public class Lights {
        public static final int STRIP_LENGTH = 29;

        public static final Distance STRIP_DENSITY = Meters.of(1 / 120.0);

        public enum ColorState {
            GREEN (0, 255, 0),
            WHITE (255, 255, 255);

    
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
        public static final int PIVOT_MOTOR_STALL_LIMIT = 10;

        public enum State {
            GROUND(-1),
            LAUNCHER(1);

            public double pos;

            private State(double pos) {
                this.pos = pos;
            }
        } 
    }

    public class Intake {
        public static final int INTAKE_MOTOR_STALL_LIMIT = 10;

        public enum State {
            INTAKE(-0.5),
            STOP(0),
            BACK(0.5);

            public double speed;

            private State(double speed) {
                this.speed = speed;
            }
        } 
    }

    public class Launcher {
        public static final int LAUNCHER_MOTOR_STALL_LIMIT = 50;
        
        public enum State {
            LAUNCH(0.8),
            STOP(0),
            BACK(-0.1);

            public double speed;

            private State(double speed) {
                this.speed = speed;
            }
        } 
    }
}
