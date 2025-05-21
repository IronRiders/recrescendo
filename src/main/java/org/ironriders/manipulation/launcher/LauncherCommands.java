package org.ironriders.manipulation.launcher;

import org.ironriders.lib.Constants;
import org.ironriders.lib.Constants.Launcher.State;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class LauncherCommands {

    private LauncherSubsystem launcher;

    public LauncherCommands(LauncherSubsystem launcher) {
        this.launcher = launcher;

        launcher.publish("Launch", this.launch());
        launcher.publish("Stop", this.set(State.STOP));

        launcher.publish("Up Speed", this.upTargetVelocity());
        launcher.publish("Down Speed", this.downTargetVelocity());
    }

    public Command set(Constants.Launcher.State state) {
        return Commands.runOnce(() -> launcher.setTargetVelocity(state.speed));
    }

    public Command stop() {
        return Commands.runOnce(() -> launcher.setTargetVelocity(0));
    }

    public Command upTargetVelocity() {
        return Commands.runOnce(() -> launcher.upVelo());
    }

    public Command downTargetVelocity() {
        return Commands.runOnce(() -> launcher.downVelo());
    }

    public Command launch() {
        return Commands.runOnce(() -> launcher.setTargetVelocity(Constants.Launcher.State.LAUNCH.speed)).andThen(Commands.waitUntil(() -> launcher.atSpeed())).andThen(
                Commands.waitSeconds(Constants.Launcher.LAUNCH_TIMEOUT).andThen(() -> launcher.setTargetVelocity(Constants.Launcher.State.STOP.speed)));
    }

    public Command launchGivenSpeed(double target) {
        return Commands.runOnce(() -> launcher.setTargetVelocity(target)).andThen(Commands.waitUntil(() -> launcher.atSpeed())).andThen(
                Commands.waitSeconds(Constants.Launcher.LAUNCH_TIMEOUT).andThen(() -> launcher.setTargetVelocity(Constants.Launcher.State.STOP.speed)));
    }
}