package org.ironriders.climber;

import org.ironriders.lib.IronSubsystem;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ClimberSubsystem extends IronSubsystem {

  private final ClimberCommands commands = new ClimberCommands(this);

  private boolean manual = false;

 public enum State {
      FORWARD(0.15),
      FAST(1),
      BACK(-0.15),
      BACKFAST(-.25),
      OFF(0);

      public double speed;

      private State(double speed) {
        this.speed = speed;
      }
    }


  private final SparkMax motor = new SparkMax(14, MotorType.kBrushless);

  ProfiledPIDController pid = new edu.wpi.first.math.controller.ProfiledPIDController(0.1, 0, 0, new Constraints(100, 100));

  public ClimberSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();

    config.smartCurrentLimit(40).idleMode(IdleMode.kBrake);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pid.reset(getExtension());
  }

  @Override
  public void periodic() {
    if (!manual) {
      motor.set(pid.calculate(getExtension()));
    }

    publish("extension", getExtension());
    publish("pid", pid);
    publish("goal", pid.getGoal().position);

    publish("current", motor.getOutputCurrent());
  }

  public void setGoal(double goal) {
    pid.setGoal(goal);
  }

  double getExtension() {
    return motor.getEncoder().getPosition();
  }

  void setHomed() {
    motor.getEncoder().setPosition(0);
  }

  void setMotor(double speed) {
    manual = true;
    motor.set(speed);
  }

  void setPIDAllowed() {
    manual = false;
  }

  public ClimberCommands getCommands() {
    return commands;
  }
}
