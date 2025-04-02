package org.ironriders.manipulation.launcher;

import org.ironriders.lib.Constants;
import org.ironriders.lib.Constants.Launcher.State;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class LauncherCommands {

    private LauncherSubsystem launcher;

    public LauncherCommands(LauncherSubsystem launcher) {
        this.launcher = launcher;

        launcher.publish("Launch", this.set(State.LAUNCH));
        launcher.publish("Stop", this.set(State.STOP));
        launcher.publish("Reverse", this.set(State.BACK));
    }

    public Command set(Constants.Launcher.State state) {
        return Commands.runOnce(() -> launcher.setMotor(state.speed));
    }
}
