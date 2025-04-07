package org.ironriders.manipulation.pivot;

import org.ironriders.lib.Constants.Identifiers;
import org.ironriders.lib.Constants.Pivot;
import org.ironriders.lib.Constants.Robot;
import org.ironriders.lib.IronSubsystem;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class PivotSubsystem extends IronSubsystem {

    private final PivotCommands commands = new PivotCommands(this);

    private final SparkMax motor = new SparkMax(Identifiers.PIVOT_MOTOR, MotorType.kBrushless);
    private final ProfiledPIDController pidControl = 
            new ProfiledPIDController(Pivot.CONTROL_P, Pivot.CONTROL_I, Pivot.CONTROL_D, Pivot.CONTROL_CONSTRAINTS);

    private final DutyCycleEncoder encoder = new DutyCycleEncoder(Identifiers.PIVOT_ENCODER);
    private final SparkLimitSwitch forwardLimitSwitch = motor.getForwardLimitSwitch();
    private final SparkLimitSwitch reverseLimitSwitch = motor.getReverseLimitSwitch();

    public PivotSubsystem() {
        SparkBaseConfig config = new SparkMaxConfig()
            .smartCurrentLimit(Pivot.MOTOR_CURRENT_LIMIT)
            .voltageCompensation(Robot.COMPENSATED_VOLTAGE)
            .idleMode(IdleMode.kBrake);
        
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pidControl.setTolerance(Pivot.CONTROL_TOLERANCE);
    }

    @Override
    public void periodic() {
        // motor.set(pidControl.calculate(encoder.getPosition() * 360 - Pivot.ENCODER_OFFSET));

        publish("Limit Switch Forward Pressed", forwardLimitSwitch.isPressed());
        publish("Limit Switch Reverse Pressed", reverseLimitSwitch.isPressed());

        publish("Goal Angle Position", pidControl.getGoal().position);
        publish("Goal Angle Velocity", pidControl.getGoal().velocity);

        publish("Current Angle", encoder.get() * 360 - Pivot.ENCODER_OFFSET);
        publish("Current Angle direct", encoder.get());

    }

    public void setGoal(double goal) {
        pidControl.setGoal(goal);
    }

    public boolean atGoal() {
        return pidControl.atGoal();
    }

    public PivotCommands getCommands() {
        return commands;
    }
}
