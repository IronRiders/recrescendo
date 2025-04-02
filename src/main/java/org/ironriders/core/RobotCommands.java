package org.ironriders.core;

import java.util.function.DoubleSupplier;

import org.ironriders.climber.ClimberCommands;
import org.ironriders.drive.DriveCommands;
import org.ironriders.manipulation.intake.IntakeCommands;
import org.ironriders.manipulation.launcher.LauncherCommands;
import org.ironriders.manipulation.pivot.PivotCommands;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotCommands {
    private final DriveCommands driveCommands;
	private final LauncherCommands launcherCommands;
	private final PivotCommands pivotCommands;
	private final IntakeCommands intakeCommands;
	private final ClimberCommands climberCommands;

	public RobotCommands(
			DriveCommands driveCommands,
            LauncherCommands launcherCommands,
			PivotCommands pivotCommands,
			IntakeCommands intakeCommands, 
            ClimberCommands climberCommands) {

		this.driveCommands = driveCommands;
		this.launcherCommands = launcherCommands;
		this.pivotCommands = pivotCommands;
		this.intakeCommands = intakeCommands;
		this.climberCommands = climberCommands;
	}

    	public Command driveTeleop(DoubleSupplier inputTranslationX, DoubleSupplier inputTranslationY,
			DoubleSupplier inputRotation) {
		return driveCommands.driveTeleop(inputTranslationX, inputTranslationY, inputRotation, true);
	}
}
