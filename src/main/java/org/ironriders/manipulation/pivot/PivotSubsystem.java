package org.ironriders.manipulation.pivot;

import org.ironriders.lib.Constants;
import org.ironriders.lib.Constants.Identifiers;
import org.ironriders.lib.Constants.Pivot;
import org.ironriders.lib.Constants.Robot;
import org.ironriders.lib.IronSubsystem;
import org.ironriders.lib.Utils;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;

public class PivotSubsystem extends IronSubsystem {

    private final PivotCommands commands = new PivotCommands(this);

    private final SparkMax motor = new SparkMax(Identifiers.PIVOT_MOTOR, MotorType.kBrushless);

    private final PIDController pidControl = new PIDController(Constants.Pivot.CONTROL_P, Constants.Pivot.CONTROL_I,
            Constants.Pivot.CONTROL_D);

    private final DutyCycleEncoder encoder = new DutyCycleEncoder(Identifiers.PIVOT_ENCODER);

    private final SparkLimitSwitch forwardLimitSwitch = motor.getForwardLimitSwitch();
    private final SparkLimitSwitch reverseLimitSwitch = motor.getReverseLimitSwitch();

    // goalSetpoint is the final goal. periodicSetpoint is a sort-of inbetween
    // setpoint generated every periodic.
    private TrapezoidProfile.State goalSetpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State periodicSetpoint = new TrapezoidProfile.State();

    private final TrapezoidProfile profile;

    public PivotSubsystem() {
        SparkBaseConfig config = new SparkMaxConfig()
                .smartCurrentLimit(Pivot.MOTOR_CURRENT_LIMIT)
                .voltageCompensation(Robot.COMPENSATED_VOLTAGE)
                .idleMode(IdleMode.kBrake);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        profile = new TrapezoidProfile(Pivot.CONTROL_CONSTRAINTS);

        pidControl.setTolerance(Pivot.CONTROL_TOLERANCE);
        // pidControl.enableContinuousInput(0, 360);
        reset();

        setGoal(getRotation());
    }

    @Override
    public void periodic() {
        periodicSetpoint = profile.calculate(Pivot.CONTROL_T, periodicSetpoint, goalSetpoint);

        double pidOutput = pidControl.calculate(getRotation(), periodicSetpoint.position);

        motor.set(pidOutput);

        // publish("Limit Switch Forward Pressed", forwardLimitSwitch.isPressed());
        // freeing up space to see other stuff
        // publish("Limit Switch Reverse Pressed", reverseLimitSwitch.isPressed());

        publish("Goal Angle Velocity", this.goalSetpoint.velocity);
        publish("Goal Angle", this.goalSetpoint.position);

        publish("PID Output", pidOutput);

        publish("Current Angle", encoder.get());

        publish("Current Angle In Deg", getRotation());

    }

    private double getRotation() {
        return Utils
                .absoluteRotation((encoder.get() * Constants.Pivot.GEAR_RATIO) * 360 - Constants.Pivot.ENCODER_OFFSET);
    }

    public void setGoal(double goal) {
        this.goalSetpoint = new TrapezoidProfile.State(goal, 0d);
    }

    public boolean atGoal() {
        return pidControl.atSetpoint();
    }

    public PivotCommands getCommands() {
        return commands;
    }

    public void reset() {
        motor.set(0);
        pidControl.reset();
    }

}
