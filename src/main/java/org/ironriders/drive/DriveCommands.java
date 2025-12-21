package org.ironriders.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.ironriders.lib.Constants.Drive;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

@Logged
public class DriveCommands {
	private final DriveSubsystem driveSubsystem;

	public DriveCommands(DriveSubsystem driveSubsystem) {
		this.driveSubsystem = driveSubsystem;

		driveSubsystem.publish("Reset Gyro", this.resetRotation());
		driveSubsystem.publish("Invert Rot", this.invertRot());
		driveSubsystem.publish("Invert Trans", this.invertTrans());

		driveSubsystem.publish("Speed X", driveSubsystem.getSwerveDrive().getFieldVelocity().vxMetersPerSecond);
		driveSubsystem.publish("Speed Y", driveSubsystem.getSwerveDrive().getFieldVelocity().vyMetersPerSecond);
	}

	public Command drive(Supplier<Translation2d> translation, DoubleSupplier rotation, BooleanSupplier fieldRelative) {
		return Commands.run(() -> {
			driveSubsystem.drive(translation.get(), rotation.getAsDouble(), fieldRelative.getAsBoolean());
		}, driveSubsystem);
	}

	public Command driveTeleop(DoubleSupplier inputTranslationX, DoubleSupplier inputTranslationY,
			DoubleSupplier inputRotation, boolean fieldRelative) {


		double invert = DriverStation.getAlliance().isEmpty()
				|| DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
						? 1
						: -1;

		return drive(
				() -> new Translation2d(inputTranslationX.getAsDouble(), inputTranslationY.getAsDouble())
						.times(Drive.SWERVE_MAX_TRANSLATION_TELEOP * invert),
				() -> inputRotation.getAsDouble() * Drive.SWERVE_MAX_ANGULAR_TELEOP * invert,
				() -> fieldRelative);
	}

	public Command resetRotation() {
		return Commands.runOnce(() -> driveSubsystem.resetRotation());
	}

	public Command invertRot() {
		return Commands.runOnce(() -> driveSubsystem.switchRotation());
	}

	public Command invertTrans() {
		return Commands.runOnce(() -> driveSubsystem.switchDrive());
	}
}
