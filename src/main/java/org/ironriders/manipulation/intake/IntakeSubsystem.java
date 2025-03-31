package org.ironriders.manipulation.intake;

import org.ironriders.lib.IronSubsystem;

public class IntakeSubsystem extends IronSubsystem {

    public IntakeCommands commands = new IntakeCommands(this);

    // init shit

    public IntakeSubsystem() {

    }

    public IntakeCommands getCommands() {
        return commands;
    }
}
