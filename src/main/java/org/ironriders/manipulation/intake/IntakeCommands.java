package org.ironriders.manipulation.intake;

import org.ironriders.lib.Constants;
import org.ironriders.lib.Constants.Intake.State;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakeCommands {

    private IntakeSubsystem intake;

    public IntakeCommands(IntakeSubsystem intake) {
        this.intake = intake;

        intake.publish("Intake", this.intake());
        intake.publish("Intake fake", this.set(State.INTAKE));

        intake.publish("Stop", this.set(State.STOP));
        intake.publish("Eject fake", this.set(State.BACK));
        intake.publish("Eject", this.eject());

    }

    public Command set(Constants.Intake.State state) {
        return Commands.runOnce(() -> intake.setMotor(state.speed));
    }

    public Command eject() {
        return Commands.runOnce(() -> intake.setMotor(Constants.Intake.State.BACK.speed))
                .andThen(Commands.waitSeconds(Constants.Intake.EJECT_WAIT_TIME))
                .andThen(() -> intake.setMotor(Constants.Intake.State.STOP.speed));
    }

    public Command intake() {
        return intake.defer(() -> {
            return new Command() {
                public void execute() {
                    intake.setMotor(Constants.Intake.State.INTAKE.speed);
                }

                public boolean isFinished() {
                    return intake.hasNote();
                }

                public void end(boolean interrupted) {
                    intake.setMotor(Constants.Intake.State.STOP.speed);
                }
            };
        });
    };
}
