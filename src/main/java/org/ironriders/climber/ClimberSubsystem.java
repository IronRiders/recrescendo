package org.ironriders.climber;

import org.ironriders.lib.IronSubsystem;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ClimberSubsystem extends IronSubsystem {

  private final ClimberCommands commands = new ClimberCommands(this);

 public enum State {
      FORWARD(0.15),
      FAST(.25),
      BACK(-0.15),
      BACKFAST(-.25),
      OFF(0)
      ;

      public double speed;

      private State(double speed) {
        this.speed = speed;
      }
    }

  private final SparkMax motor = new SparkMax(14, MotorType.kBrushless);

  public ClimberSubsystem() {
      
    SparkMaxConfig config = new SparkMaxConfig();

    config.smartCurrentLimit(40).idleMode(IdleMode.kBrake);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void set(State goal){
    motor.set(goal.speed);
  }

  public ClimberCommands getCommands() {
    return commands;
  }
}
