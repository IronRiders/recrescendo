package org.ironriders.manipulation.intake;

import org.ironriders.lib.Constants;
import org.ironriders.lib.IronSubsystem;

import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfig.Presets;

public class IntakeSubsystem extends IronSubsystem {

  private final IntakeCommands commands = new IntakeCommands(this);

  private final SparkMax motor = new SparkMax(Constants.Identifiers.INTAKE_MOTOR, MotorType.kBrushless);

  private final LimitSwitchConfig limitSwitchConfig = new LimitSwitchConfig();

  private final SparkLimitSwitch hasNoteLimitSwitch = motor.getForwardLimitSwitch();

  public IntakeSubsystem() {
    SparkMaxConfig motorConfig = new SparkMaxConfig();

    motorConfig
        .idleMode(IdleMode.kCoast)
        .inverted(true);

    limitSwitchConfig.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);

    motorConfig.apply(limitSwitchConfig);
    motor.configure(Presets.REV_NEO.apply(motorConfig), com.revrobotics.ResetMode.kNoResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    publish("Has note", hasNoteLimitSwitch.isPressed());
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
