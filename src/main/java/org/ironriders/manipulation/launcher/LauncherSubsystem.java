package org.ironriders.manipulation.launcher;

import org.ironriders.lib.IronSubsystem;

public class LauncherSubsystem extends IronSubsystem {

    public LauncherCommands commands = new LauncherCommands(this);

    // init shit

    public LauncherSubsystem() {

    }

    public LauncherCommands getCommands() {
        return commands;
    }
}
