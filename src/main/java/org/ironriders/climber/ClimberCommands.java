package org.ironriders.climber;

import org.ironriders.climber.ClimberSubsystem.State;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ClimberCommands {

  private ClimberSubsystem climber;

  public ClimberCommands(ClimberSubsystem climber) {
   this.climber = climber;

   climber.publish("set OFF", set(State.OFF));
   climber.publish("set Forward", set(State.FORWARD));
   climber.publish("set Back", set(State.BACK));
   climber.publish("set FAST Forward", set(State.FAST));
   climber.publish("set FAST Backward", set(State.BACKFAST));
    
  }


public Command set(State goal){
    return Commands.runOnce(()->climber.set(goal));
}

}
