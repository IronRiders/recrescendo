package org.ironriders.manipulation.pivot;

import org.ironriders.lib.Constants.Pivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class PivotCommands {

    private PivotSubsystem pivot;

    public PivotCommands(PivotSubsystem pivot) {
        this.pivot = pivot;

        pivot.publish("Set to GROUND", set(Pivot.State.GROUND));
        pivot.publish("Set to LAUNCHER", set(Pivot.State.LAUNCHER));
        pivot.publish("Set to STOW", set(Pivot.State.STOWED));
    }

    /**
     * Command to set the pivot goal, then wait until that goal has been reached.
     */
    public Command set(Pivot.State state) {
        return Commands.runOnce(() -> {
            pivot.setGoal(state.position);
        }).andThen(Commands.waitUntil(pivot::atGoal));
    }
}
