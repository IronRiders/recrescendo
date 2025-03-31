package org.ironriders.lib;

import java.io.File;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

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

    public class Pivot {

    }

    public class Intake {

    }

    public class Launcher {

    }
}
