// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ironriders.core;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.ironriders.climber.ClimberCommands;
import org.ironriders.climber.ClimberSubsystem;
import org.ironriders.drive.DriveCommands;
import org.ironriders.drive.DriveSubsystem;
import org.ironriders.lib.Constants;
import org.ironriders.lib.Constants.Drive;
import org.ironriders.lib.Constants.Drive.Controller;
import org.ironriders.lib.Utils;
import org.ironriders.lights.LightsCommands;
import org.ironriders.lights.LightsSubsystem;
import org.ironriders.manipulation.intake.IntakeCommands;
import org.ironriders.manipulation.intake.IntakeSubsystem;
import org.ironriders.manipulation.launcher.LauncherCommands;
import org.ironriders.manipulation.launcher.LauncherSubsystem;
import org.ironriders.manipulation.pivot.PivotCommands;
import org.ironriders.manipulation.pivot.PivotSubsystem;
import org.ironriders.vision.VisionCommands;
import org.ironriders.vision.VisionSubsystem;

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

  public VisionSubsystem visionSubsystem = new VisionSubsystem();
  public VisionCommands visionCommands = visionSubsystem.getCommands();

  public Command activeCommand;

  public double speedMultiplier = 1;
  public double angleMultiplier = 1;
  private final SendableChooser<Command> autoChooser;

  private final CommandXboxController primaryController =
      new CommandXboxController(Constants.Identifiers.CONTROLLER_PRIMARY_PORT);

  public RobotCommands robotCommands = new RobotCommands(driveCommands,
      launcherCommands, pivotCommands, intakeCommands, climberCommands);

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Select", autoChooser);

    DriverStation.silenceJoystickConnectionWarning(true);

    configureBindings();
  }

  private void checkControl() {
    double average = (Math.abs(primaryController.getLeftY())
                         + Math.abs(primaryController.getLeftX())
                         + Math.abs(primaryController.getRightX()))
        / 3d;

    SmartDashboard.putNumber("RobotContainer/average", average);

    if (average > Drive.DRIVE_OVERRIDE_THRESHOLD) {
      DriveSubsystem.setController(Controller.DRIVER);
      DriveSubsystem.cancelPathfind();
    }
  }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(Commands.parallel(
        robotCommands
            .driveTeleop(
                ()
                    -> Utils.controlCurve(-primaryController.getLeftY(),
                           Constants.Drive.TRANSLATION_CONTROL_EXPONENT,
                           Constants.Drive.TRANSLATION_CONTROL_DEADBAND)
                    * speedMultiplier,
                ()
                    -> Utils.controlCurve(-primaryController.getLeftX(),
                           Constants.Drive.TRANSLATION_CONTROL_EXPONENT,
                           Constants.Drive.TRANSLATION_CONTROL_DEADBAND)
                    * speedMultiplier,
                ()
                    -> Utils.controlCurve(-primaryController.getRightX(),
                           Constants.Drive.ROTATION_CONTROL_EXPONENT,
                           Constants.Drive.ROTATION_CONTROL_DEADBAND)
                    * angleMultiplier)
            .withName("Drive Teleop"),
        Commands.run(this::checkControl)));

    primaryController.rightTrigger()
        .onTrue(activeCommand = robotCommands.intake())
        .onFalse(robotCommands.launch().unless(
            () -> !intakeSubsystem.hasNote())); // intake waits for a note and then moves to position, launch ejects from 
                                                // the manipulator and spins up the launcher for 0.4 (might have changed) second(s)

    primaryController.leftTrigger().onTrue(robotCommands.launch());

    primaryController.x().onTrue(Commands.parallel(
        Commands.runOnce(() -> activeCommand.cancel()), robotCommands.reset()));

    primaryController.b().onTrue(launcherCommands.set(
        Constants.Launcher.State.STOP)); // force stop launcher

    primaryController.y().onTrue(robotCommands.eject().unless(
        () -> !intakeSubsystem.hasNote())); // eject unless we don't have a note

    primaryController.a()
        .onTrue(driveCommands.setController(
            Controller.VISION)) // Give control of the drive
                                // system to vision
        .onFalse(driveCommands.setController(Controller.DRIVER));

    primaryController.povUp().onTrue(launcherCommands.upTargetVelocity());
    primaryController.povDown().onTrue(launcherCommands.downTargetVelocity());

    primaryController.povRight().onTrue(
        Commands.runOnce(() -> speedMultiplier += 0.5));
    primaryController.povLeft().onTrue(
        Commands.runOnce(() -> speedMultiplier -= 0.5));
  }

  /**
   * Get command configured in auto chooser.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
