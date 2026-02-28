// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.ironriders.core;

import java.text.FieldPosition;
import java.util.Optional;

import org.ironriders.climber.ClimberCommands;
import org.ironriders.climber.ClimberSubsystem;
import org.ironriders.drive.DriveCommands;
import org.ironriders.drive.DriveSubsystem;
import org.ironriders.lib.Constants;
import org.ironriders.lib.DriverRequest;
import org.ironriders.lib.DriverRequest.AlignTargetingMode;
import org.ironriders.lib.DriverRequest.PriorityMode;
import org.ironriders.lib.Utils;
import org.ironriders.lib.field.FieldPositions;
import org.ironriders.lib.field.Zone;
import org.ironriders.lib.field.FieldElement.ElementType;
import org.ironriders.lib.field.Zone.ZoneType;
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

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    public static Zone passingZone = new Zone(ZoneType.PASSING);
    public static Zone scoringZone = new Zone(ZoneType.SCORING);

    public static Pose2d ClimbingOutpostPose = new Pose2d(0,0, new Rotation2d(0));
    public static Pose2d ClimbingDepotPose = new Pose2d();

    public static DriveSubsystem driveSubsystem = new DriveSubsystem();
    public static DriveCommands driveCommands = driveSubsystem.getCommands();

    public static PivotSubsystem pivotSubsystem = new PivotSubsystem();
    public static PivotCommands pivotCommands = pivotSubsystem.getCommands();

    public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public static IntakeCommands intakeCommands = intakeSubsystem.getCommands();

    public static LauncherSubsystem launcherSubsystem = new LauncherSubsystem();
    public static LauncherCommands launcherCommands = launcherSubsystem.getCommands();

    public static ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    public static ClimberCommands climberCommands = climberSubsystem.getCommands();

    public LightsSubsystem lightsSubsystem = new LightsSubsystem();
    public LightsCommands lightsCommands = lightsSubsystem.getCommands();

    public VisionSubsystem visionSubsystem = new VisionSubsystem();
    public VisionCommands visionCommands = visionSubsystem.getCommands();

    private final SendableChooser<Command> autoChooser;

    private static boolean targetingHub = false;
    private static boolean targetingPassing = false;

    public static final CommandXboxController primaryController = new CommandXboxController(
            Constants.Identifiers.CONTROLLER_PRIMARY_PORT);

    public static RobotCommands robotCommands = new RobotCommands(driveCommands,
            launcherCommands, pivotCommands, intakeCommands, climberCommands);

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Select", autoChooser);
        SmartDashboard.putData("Target Hub", TargetingControl.targetHub());
        SmartDashboard.putData("revert Targeting",  Commands.runOnce(()->RobotContainer.revertToSafeDefaults())); //this might cause subsystem conflicts

        DriverStation.silenceJoystickConnectionWarning(true);

        passingZone = new Zone(ZoneType.PASSING);
        scoringZone = new Zone(ZoneType.SCORING);

        configureBindings();
    }

    public static void init() {
        CommandScheduler.getInstance().schedule(robotCommands.reset());
    }

    public static Optional<Zone> getCurrentZone() {
        if (passingZone.inside()) {
            return Optional.of(passingZone);
        } else if (scoringZone.inside()) {
            return Optional.of(scoringZone);
        }

        return Optional.empty();
    }

    public static void revertToSafeDefaults() {
        targetingHub = false;
        targetingPassing = false;
        TargetingControl.revertToSafeDefaults();
    }

    private void configureBindings() {
        driveSubsystem.setDefaultCommand(Commands.parallel(
                robotCommands
                        .driveTeleop(
                                () -> Utils.controlCurve(primaryController.getLeftY(),
                                        Constants.Drive.TRANSLATION_CONTROL_EXPONENT,
                                        Constants.Drive.TRANSLATION_CONTROL_DEADBAND),
                                () -> Utils.controlCurve(primaryController.getLeftX(),
                                        Constants.Drive.TRANSLATION_CONTROL_EXPONENT,
                                        Constants.Drive.TRANSLATION_CONTROL_DEADBAND),
                                () -> Utils.controlCurve(primaryController.getRightX(),
                                        Constants.Drive.ROTATION_CONTROL_EXPONENT,
                                        Constants.Drive.ROTATION_CONTROL_DEADBAND))
                        .withName("Drive Teleop")));

        primaryController.a().onTrue(
                new InstantCommand(() -> {
                    targetingHub = !targetingHub;
                    if (targetingHub) {
                        targetingPassing = false;
                        TargetingControl.targetHubInternal();
                    } else {
                        revertToSafeDefaults();
                    }
                }));

        primaryController.x().onTrue(
                new InstantCommand(() -> {
                    targetingPassing = !targetingPassing;
                    if (targetingPassing) {
                        targetingHub = false;
                        TargetingControl.targetPassingInternal();
                    } else {
                        revertToSafeDefaults();
                    }
                }));
                

        // --- Align ---
        primaryController.y()
                .onTrue(Commands
                        .runOnce(() -> {
                            new DriverRequest(PriorityMode.ALIGN_PRIORITY, AlignTargetingMode.OUTPOST)
                                    .send("align outpost");
                            targetingHub = false;
                            targetingPassing = false;
                        }))
                .onFalse(Commands.runOnce(() -> revertToSafeDefaults()));

        primaryController.b()
                .onTrue(Commands
                        .runOnce(() -> {
                            new DriverRequest(PriorityMode.ALIGN_PRIORITY, AlignTargetingMode.BUMP).send("align bump");
                            targetingHub = false;
                            targetingPassing = false;
                        }))
                .onFalse(Commands.runOnce(() -> revertToSafeDefaults()));

        primaryController.leftBumper().onTrue(Commands.runOnce(() -> {
            CommandScheduler.getInstance().schedule(DriveSubsystem
                    .pathfindThenFlipPathIfBetterThenFollow(DriveSubsystem.loadPath("Center Sweep").get()));
        }));

        // Line up to score
        primaryController.rightBumper().onTrue(Commands.runOnce(() -> {
            CommandScheduler.getInstance().schedule(DriveSubsystem.pathfindToPoseThenAimAt(scoringZone.centerPoint(), FieldPositions.get(ElementType.HUB).toPose2d()));
        }));

        primaryController.povRight().onTrue(Commands.runOnce(()-> {
            CommandScheduler.getInstance().schedule(DriveSubsystem.pathfindToPose(ClimbingOutpostPose));
        }));
        
        primaryController.povDownLeft().onTrue(Commands.runOnce(()-> {
            CommandScheduler.getInstance().schedule(DriveSubsystem.pathfindToPose(ClimbingOutpostPose));
        }));
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
