// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ironriders.core;

import org.ironriders.climber.ClimberCommands;
import org.ironriders.climber.ClimberSubsystem;
import org.ironriders.drive.DriveCommands;
import org.ironriders.drive.DriveSubsystem;
import org.ironriders.lib.Constants;
import org.ironriders.lib.Utils;
import org.ironriders.lights.LightsCommands;
import org.ironriders.lights.LightsSubsystem;
import org.ironriders.manipulation.intake.IntakeCommands;
import org.ironriders.manipulation.intake.IntakeSubsystem;
import org.ironriders.manipulation.launcher.LauncherCommands;
import org.ironriders.manipulation.launcher.LauncherSubsystem;
import org.ironriders.manipulation.pivot.PivotCommands;
import org.ironriders.manipulation.pivot.PivotSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

	public DriveSubsystem driveSubsystem = new DriveSubsystem();
	public DriveCommands driveCommands = driveSubsystem.getCommands();

	public PivotSubsystem pivotSubsystem = new PivotSubsystem();
	public PivotCommands pivotCommands = pivotSubsystem.getCommands();

	public IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
	public IntakeCommands intakeCommands = intakeSubsystem.getCommands();

	public LauncherSubsystem launcherSubsystem = new LauncherSubsystem();
	public LauncherCommands launcherCommands = launcherSubsystem.getCommands();

	public ClimberSubsystem climberSubsystem = new ClimberSubsystem();
	public ClimberCommands climberCommands = climberSubsystem.getCommands();

	public LightsSubsystem lightsSubsystem = new LightsSubsystem();
	public LightsCommands lightsCommands = lightsSubsystem.getCommands();

	public Command activeCommand;

	private final CommandXboxController primaryController = new CommandXboxController(
			Constants.Identifiers.CONTROLLER_PRIMARY_PORT);

	public RobotCommands robotCommands = new RobotCommands(
			driveCommands,
			launcherCommands,
			pivotCommands,
			intakeCommands,
			climberCommands);

	public RobotContainer() {
		configureBindings();
	}

	private void configureBindings() {
		driveSubsystem.setDefaultCommand(
				robotCommands.driveTeleop(
						() -> Utils.controlCurve(
								-primaryController.getLeftY() * driveSubsystem.ControlSpeedMultipler
										* driveSubsystem.getinversionStatus(),
								Constants.Drive.TRANSLATION_CONTROL_EXPONENT,
								Constants.Drive.TRANSLATION_CONTROL_DEADBAND),
						() -> Utils.controlCurve(
								-primaryController.getLeftX() * driveSubsystem.ControlSpeedMultipler
										* driveSubsystem.getinversionStatus(),
								Constants.Drive.TRANSLATION_CONTROL_EXPONENT,
								Constants.Drive.TRANSLATION_CONTROL_DEADBAND),
						() -> Utils.controlCurve(
								primaryController.getRightX() * driveSubsystem.ControlSpeedMultipler
										* driveSubsystem.getinversionStatus(),
								Constants.Drive.ROTATION_CONTROL_EXPONENT,
								Constants.Drive.ROTATION_CONTROL_DEADBAND)));

		primaryController.rightTrigger().onFalse(activeCommand = robotCommands.intake())
				.onFalse(robotCommands.launch()); // intake waits for a note and then moves to position, launch ejects
													// from the manipulator and spins up the launcher for 1 (might have
													// changed) second(s)

		primaryController.x().onTrue(Commands.runOnce(() -> activeCommand.cancel())); // cancel the launch

		primaryController.b().onTrue(launcherCommands.set(Constants.Launcher.State.STOP)); // force stop launcher

		primaryController.y().onTrue(robotCommands.eject().unless(() -> !intakeSubsystem.hasNote())); // eject unless we
																										// don't have a
																										// note

		primaryController.a().onTrue(robotCommands.reset()); // reset everything
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
