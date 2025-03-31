package org.ironriders.manipulation.pivot;

import org.ironriders.lib.IronSubsystem;

public class PivotSubsystem extends IronSubsystem {

    public PivotCommands commands = new PivotCommands(this);

    // init shit

    public PivotSubsystem() {

    }

    public PivotCommands getCommands() {
        return commands;
    }
}
