// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This command takes operator control from the driver and controls the swerve
 * motors with that control signal.
 */
public class TeleopDriveCommand extends Command {
	private double SpeedLimit = 0.9;
	private DriveSubsystem driveSubsystem;
	private DoubleSupplier xSpeedSupplier;
	private DoubleSupplier ySpeedSupplier;
	private DoubleSupplier rotSpeedSupplier;
	private DoubleSupplier rotationAngle;
	private ProfiledPIDController rotationController = new ProfiledPIDController(DriveSubsystem.DriveConstants.kAimP,
			DriveSubsystem.DriveConstants.kAimI, DriveSubsystem.DriveConstants.kAimD,
			DriveSubsystem.DriveConstants.kAimProfile);

	/**
	 * Creates a new TeleopDriveCommand Command.
	 * 
	 * @param driveSubsystem   The drive subsystem.
	 * @param xSpeedSupplier   The x speed.
	 * @param ySpeedSupplier   The y speed.
	 * @param rotSpeedSupplier The angular speed.
	 * @param rotationAngle    The rotation angle.
	 */
	public TeleopDriveCommand(DriveSubsystem driveSubsystem,
			DoubleSupplier xSpeedSupplier,
			DoubleSupplier ySpeedSupplier,
			DoubleSupplier rotSpeedSupplier,
			Double rotationAngle) {
		this(driveSubsystem, xSpeedSupplier, ySpeedSupplier, rotSpeedSupplier, () -> rotationAngle);
	}

	/**
	 * Creates a new TeleopDriveCommand Command.
	 * 
	 * @param driveSubsystem   The drive subsystem.
	 * @param xSpeedSupplier   The x speed.
	 * @param ySpeedSupplier   The y speed.
	 * @param rotSpeedSupplier The angular speed.
	 * @param stageSide        LEFT or RIGHT
	 */
	public TeleopDriveCommand(DriveSubsystem driveSubsystem,
			DoubleSupplier xSpeedSupplier,
			DoubleSupplier ySpeedSupplier,
			DoubleSupplier rotSpeedSupplier) {
		this(driveSubsystem, xSpeedSupplier, ySpeedSupplier, rotSpeedSupplier, () -> {
			Optional<Alliance> currentAlliance = DriverStation.getAlliance();

			return 0.0;
		});
	}

	/**
	 * Creates a new TeleopDriveCommand Command.
	 * 
	 * @param driveSubsystem   The drive subsystem.
	 * @param xSpeedSupplier   The x speed.
	 * @param ySpeedSupplier   The y speed.
	 * @param rotSpeedSupplier The angular speed.
	 * @param rotationAngle    The rotation angle.
	 */
	public TeleopDriveCommand(DriveSubsystem driveSubsystem,
			DoubleSupplier xSpeedSupplier,
			DoubleSupplier ySpeedSupplier,
			DoubleSupplier rotSpeedSupplier,
			DoubleSupplier rotationAngle) {
		this.driveSubsystem = driveSubsystem;
		addRequirements(driveSubsystem);
		this.xSpeedSupplier = xSpeedSupplier;
		this.ySpeedSupplier = ySpeedSupplier;
		this.rotSpeedSupplier = rotSpeedSupplier;
		this.rotationAngle = rotationAngle;
		rotationController.setTolerance(Math.PI / 360);
		rotationController.enableContinuousInput(0.0, 2 * Math.PI);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		rotationController.reset(Math.toRadians(driveSubsystem.getHeading()), 0);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		double rotationGoal = rotationAngle.getAsDouble();
		if(Robot.isSimulation()) {
				driveSubsystem.moveSimulation(xSpeedSupplier.getAsDouble(), ySpeedSupplier.getAsDouble(), -3 * this.rotSpeedSupplier.getAsDouble());
		}if (rotationGoal != -1.0) {
			rotationController.setGoal(rotationGoal);
			driveSubsystem.drive(-xSpeedSupplier.getAsDouble(), -ySpeedSupplier.getAsDouble(),
					rotationController.calculate(Math.toRadians(driveSubsystem.getHeading())),
					true);
		} else {
			driveSubsystem.drive(-xSpeedSupplier.getAsDouble() * SpeedLimit, -ySpeedSupplier.getAsDouble() * SpeedLimit,
					-rotSpeedSupplier.getAsDouble(),
					true);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		driveSubsystem.drive(0, 0, 0, false);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
