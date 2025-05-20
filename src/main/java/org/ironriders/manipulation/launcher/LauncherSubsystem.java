package org.ironriders.manipulation.launcher;

import org.ironriders.lib.Constants;
import org.ironriders.lib.IronSubsystem;
import org.ironriders.lib.Utils;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;

public class LauncherSubsystem extends IronSubsystem {

    private final LauncherCommands commands = new LauncherCommands(this);

    private final SparkMax primaryMotor = new SparkMax(Constants.Identifiers.LAUNCHER_MOTOR_RIGHT,
            MotorType.kBrushless); // lead motor is the right one
    private final SparkMax followerMotor = new SparkMax(Constants.Identifiers.LAUNCHER_MOTOR_LEFT,
            MotorType.kBrushless);

    private final PIDController primaryController = new PIDController(Constants.Launcher.CONTROL_P,
            Constants.Launcher.CONTROL_I, Constants.Launcher.CONTROL_D);
    private final PIDController followerController = new PIDController(Constants.Launcher.CONTROL_P,
            Constants.Launcher.CONTROL_I, Constants.Launcher.CONTROL_D);

    private double targetVelocity = 0.0; // in RPM

    public LauncherSubsystem() {
        SparkMaxConfig primaryConfig = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();

        primaryConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(Constants.Launcher.LAUNCHER_MOTOR_STALL_LIMIT)
                .inverted(false);

        followerConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(Constants.Launcher.LAUNCHER_MOTOR_STALL_LIMIT)
                .inverted(true);

        primaryMotor.configure(primaryConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void setMotors(double speed) {
        primaryMotor.set(speed);
        followerMotor.set(speed);
    }

    public void setTargetVelocity(double target) {
        targetVelocity = target;
    }

    public void upVelo() {
        targetVelocity += 10;
    }

    public void downVelo() {
        targetVelocity -= 10;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public boolean atSpeed() {
        if (Utils.inRange(targetVelocity - Constants.Launcher.CONTROL_SPEED_TOLERANCE,
                targetVelocity + Constants.Launcher.CONTROL_SPEED_TOLERANCE, primaryMotor.getEncoder().getVelocity())
                && Utils.inRange(targetVelocity - Constants.Launcher.CONTROL_SPEED_TOLERANCE,
                        targetVelocity + Constants.Launcher.CONTROL_SPEED_TOLERANCE,
                        followerMotor.getEncoder().getVelocity())) {
            //return true;
            return false;
        }
        return false;
    }

    @Override
    public void periodic() {
        publish("Right Velocity", primaryMotor.getEncoder().getVelocity());
        publish("Left Velocity", followerMotor.getEncoder().getVelocity());
        publish("Target Velocity", targetVelocity);

        if (targetVelocity == 0) {
            setMotors(0);
        } else {
            followerMotor.set(followerController.calculate(followerMotor.getEncoder().getVelocity()));
            primaryMotor.set(primaryController.calculate(primaryMotor.getEncoder().getVelocity()));
        }
    }

    public LauncherCommands getCommands() {
        return commands;
    }
}
