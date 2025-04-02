package org.ironriders.manipulation.launcher;

import org.ironriders.lib.Constants;
import org.ironriders.lib.IronSubsystem;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class LauncherSubsystem extends IronSubsystem {

    public LauncherCommands commands = new LauncherCommands(this);

    private final SparkMax primaryMotor = new SparkMax(Constants.Identifiers.LAUNCHER_MOTOR_RIGHT, MotorType.kBrushless); // lead motor is the right one
    private final SparkMax followerMotor = new SparkMax(Constants.Identifiers.LAUNCHER_MOTOR_LEFT, MotorType.kBrushless);

    public LauncherSubsystem() {
        SparkMaxConfig primaryConfig = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();

        primaryConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(Constants.Launcher.LAUNCHER_MOTOR_STALL_LIMIT)
                .inverted(true);

        followerConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(Constants.Launcher.LAUNCHER_MOTOR_STALL_LIMIT)
                .follow(Constants.Identifiers.LAUNCHER_MOTOR_RIGHT, true);

        primaryMotor.configure(primaryConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void setMotor(double speed) {
        primaryMotor.set(speed);
    }

    public LauncherCommands getCommands() {
        return commands;
    }
}
