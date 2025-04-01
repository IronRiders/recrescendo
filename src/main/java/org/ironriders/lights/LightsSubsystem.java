package org.ironriders.lights;

import org.ironriders.lib.IronSubsystem;

public class LightsSubsystem extends IronSubsystem {

    public LightsCommands commands = new LightsCommands(this);

    // init shit

    public LightsSubsystem() {

    }

    public LightsCommands getCommands() {
        return commands;
    }
}
