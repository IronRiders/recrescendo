// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ironriders.core;

import org.ironriders.climber.ClimberCommands;
import org.ironriders.climber.ClimberSubsystem;
import org.ironriders.drive.DriveCommands;
import org.ironriders.drive.DriveSubsystem;
import org.ironriders.lib.Constants;
import org.ironriders.manipulation.intake.IntakeCommands;
import org.ironriders.manipulation.intake.IntakeSubsystem;
import org.ironriders.manipulation.launcher.LauncherCommands;
import org.ironriders.manipulation.launcher.LauncherSubsystem;
import org.ironriders.manipulation.pivot.PivotCommands;
import org.ironriders.manipulation.pivot.PivotSubsystem;
import  org.ironriders.lib.Utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
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

	private final CommandXboxController primaryController = new CommandXboxController(
		Constants.Identifiers.CONTROLLER_PRIMARY_PORT);
	private final CommandGenericHID secondaryController = new CommandJoystick(
		Constants.Identifiers.CONTROLLER_SECONDARY_PORT);

	public RobotCommands robotCommands = new RobotCommands(
		driveCommands,
		launcherCommands,
		pivotCommands,
		intakeCommands, 
		climberCommands
	);

	public RobotContainer() {
		configureBindings();
	}

	private void configureBindings() {
		// DRIVE CONTROLS
		driveSubsystem.setDefaultCommand(
				robotCommands.driveTeleop(
						() -> Utils.controlCurve(
								-primaryController.getLeftY()* driveSubsystem.ControlSpeedMultipler *driveSubsystem.getinversionStatus(),
								Constants.Drive.TRANSLATION_CONTROL_EXPONENT,
								Constants.Drive.TRANSLATION_CONTROL_DEADBAND),
						() -> Utils.controlCurve(
								-primaryController.getLeftX()* driveSubsystem.ControlSpeedMultipler *driveSubsystem.getinversionStatus(),
								Constants.Drive.TRANSLATION_CONTROL_EXPONENT,
								Constants.Drive.TRANSLATION_CONTROL_DEADBAND),
						() -> Utils.controlCurve(
								primaryController.getRightX()* driveSubsystem.ControlSpeedMultipler *driveSubsystem.getinversionStatus(),
								Constants.Drive.ROTATION_CONTROL_EXPONENT,
								Constants.Drive.ROTATION_CONTROL_DEADBAND)));
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
