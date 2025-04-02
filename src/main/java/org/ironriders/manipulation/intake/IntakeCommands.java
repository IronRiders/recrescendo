package org.ironriders.manipulation.intake;

import org.ironriders.lib.Constants;
import org.ironriders.lib.Constants.Intake.State;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakeCommands {

    private IntakeSubsystem intake;

    public IntakeCommands(IntakeSubsystem intake) {
        this.intake = intake;

        intake.publish("Intake", this.set(State.INTAKE));
        intake.publish("Stop", this.set(State.STOP));
        intake.publish("Reverse", this.set(State.BACK));
    }

    public Command set(Constants.Intake.State state) {
        return Commands.runOnce(() -> intake.setMotor(state.speed));
    }
}
