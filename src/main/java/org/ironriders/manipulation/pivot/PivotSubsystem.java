package org.ironriders.manipulation.pivot;

import org.ironriders.lib.Constants.Identifiers;
import org.ironriders.lib.Constants.Pivot;
import org.ironriders.lib.Constants.Robot;
import org.ironriders.lib.IronSubsystem;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class PivotSubsystem extends IronSubsystem {

    private final PivotCommands commands = new PivotCommands(this);

    private final SparkMax motor = new SparkMax(Identifiers.PIVOT_MOTOR, MotorType.kBrushless);

    private final SparkAbsoluteEncoder encoder = motor.getAbsoluteEncoder();

    private final SparkLimitSwitch forwardLimitSwitch = motor.getForwardLimitSwitch();
    private final SparkLimitSwitch reverseLimitSwitch = motor.getReverseLimitSwitch();

    // init shit

    public PivotSubsystem() {

        SparkBaseConfig config = new SparkMaxConfig()
            .smartCurrentLimit(Pivot.MOTOR_CURRENT_LIMIT)
            .voltageCompensation(Robot.COMPENSATED_VOLTAGE)
            .idleMode(IdleMode.kBrake);
        
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }



    public PivotCommands getCommands() {
        return commands;
    }
}
