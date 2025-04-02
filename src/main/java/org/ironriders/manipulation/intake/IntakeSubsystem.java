package org.ironriders.manipulation.intake;

import org.ironriders.lib.Constants;
import org.ironriders.lib.IronSubsystem;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeSubsystem extends IronSubsystem {

    public IntakeCommands commands = new IntakeCommands(this);

    private final SparkMax motor = new SparkMax(Constants.Identifiers.INTAKE_MOTOR, MotorType.kBrushless); 

    private final SparkLimitSwitch hasNoteLimitSwitch = motor.getReverseLimitSwitch();

    public IntakeSubsystem() {
        SparkMaxConfig motorConfig = new SparkMaxConfig();

        motorConfig.idleMode(IdleMode.kCoast)
        .smartCurrentLimit(Constants.Intake.INTAKE_MOTOR_STALL_LIMIT)
        .inverted(true);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setMotor(double speed) {
        motor.set(speed);
    }

    public SparkLimitSwitch geLimitSwitch() {
        return hasNoteLimitSwitch;
    }

    public boolean hasNote() {
        return hasNoteLimitSwitch.isPressed();
    }

    public IntakeCommands getCommands() {
        return commands;
    }
}
