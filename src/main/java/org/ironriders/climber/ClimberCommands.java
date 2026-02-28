package org.ironriders.climber;

import org.ironriders.climber.ClimberSubsystem.State;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ClimberCommands {

  private ClimberSubsystem climber;

  public ClimberCommands(ClimberSubsystem climber) {
   this.climber = climber;

   climber.publish("zero", zeroClimber());
   climber.publish("set goal 60", setGoal(-60));
   climber.publish("set goal 0", setGoal(0));

   climber.publish("down", setManual(1));
   climber.publish("up", setManual(-1));

   climber.publish("up slow", setManual(-0.1));
   climber.publish("down slow", setManual(0.1));

    climber.publish("stop", setManual(0));

   climber.publish("PID enabled", setPIDallowed());
  }

  public Command zeroClimber() {
    return Commands.runOnce(()->climber.setHomed());
  }

  public Command setGoal(double goal) {
    return Commands.runOnce(()->climber.setGoal(goal));
  }

  public Command setManual(double speed) {
    return Commands.runOnce(()->climber.setMotor(speed));
  }

  public Command setPIDallowed() {
    return Commands.runOnce(()->climber.setPIDAllowed());
  }
}
