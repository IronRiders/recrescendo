package org.ironriders.manipulation.pivot;

import org.ironriders.lib.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class PivotCommands {

    private PivotSubsystem pivot;

    public PivotCommands(PivotSubsystem pivot) {
        this.pivot = pivot;
    }

    public Command set(Constants.Pivot.State state) {
        return Commands.runOnce(() -> System.out.println("hi :3 (please implement pivot)"));
    }
}
