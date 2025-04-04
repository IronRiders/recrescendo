package org.ironriders.climber;

import org.ironriders.lib.IronSubsystem;

public class ClimberSubsystem extends IronSubsystem {

    private final ClimberCommands commands = new ClimberCommands(this);

    // init shit

    public ClimberSubsystem() {

    }

    public ClimberCommands getCommands() {
        return commands;
    }
}
