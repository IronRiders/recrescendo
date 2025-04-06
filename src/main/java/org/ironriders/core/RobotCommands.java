package org.ironriders.core;

import java.util.function.DoubleSupplier;

import org.ironriders.climber.ClimberCommands;
import org.ironriders.drive.DriveCommands;
import org.ironriders.lib.Constants.Intake;
import org.ironriders.lib.Constants.Launcher;
import org.ironriders.lib.Constants.Pivot;
import org.ironriders.lib.Constants.Launcher.State;
import org.ironriders.manipulation.intake.IntakeCommands;
import org.ironriders.manipulation.launcher.LauncherCommands;
import org.ironriders.manipulation.pivot.PivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotCommands {

	private final DriveCommands driveCommands;

	private final PivotCommands pivotCommands;
	private final IntakeCommands intakeCommands;
	private final LauncherCommands launcherCommands;

	private final ClimberCommands climberCommands; // probably wont exist

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

	public Command intake() {
		return Commands.sequence(
				pivotCommands.set(Pivot.State.GROUND),
				intakeCommands.intake(),
				Commands.parallel(launcherCommands.set(Launcher.State.LAUNCH), intakeCommands.center()),
				pivotCommands.set(Pivot.State.LAUNCHER));
	}

	public Command launch() {
		return Commands.sequence(
				launcherCommands.set(Launcher.State.LAUNCH), //should already be true but anyway
				pivotCommands.set(Pivot.State.LAUNCHER), // same with this
				intakeCommands.eject(),
				launcherCommands.launch()); // launches with timeout
				
	}

	public Command eject() {
		return Commands.sequence(
				launcherCommands.set(Launcher.State.STOP),
				pivotCommands.set(Pivot.State.GROUND),
				intakeCommands.eject());
	}

	public Command reset() {
		return Commands.parallel(
				pivotCommands.set(Pivot.State.LAUNCHER),
				intakeCommands.set(Intake.State.STOP),
				launcherCommands.set(Launcher.State.STOP));
	}
}
