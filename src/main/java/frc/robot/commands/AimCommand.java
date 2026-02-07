// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterHoodSubsystem;
import frc.robot.subsystems.ShooterHoodSubsystem.ShooterAngleConstants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.TurretSubsystem.TurretConstants;
import frc.robot.utils.ShotCalculator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimCommand extends Command {
	/** Creates a new AimCommand. */
	private TurretSubsystem turretSubsystem;
	private DriveSubsystem driveSubsystem;
	private ShooterHoodSubsystem shooterHoodSubsystem;
	private ShotCalculator shotCalculator;

	public AimCommand(TurretSubsystem turretSubsystem, DriveSubsystem driveSubsystem,
			ShooterHoodSubsystem shooterHoodSubsystem, ShotCalculator shotCalculator) {
		this.turretSubsystem = turretSubsystem;
		this.driveSubsystem = driveSubsystem;
		this.shooterHoodSubsystem = shooterHoodSubsystem;
		this.shotCalculator = shotCalculator;
		addRequirements(turretSubsystem, shooterHoodSubsystem);

	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		Translation3d target = TurretSubsystem.getHub();

		if (driveSubsystem.getPose().getTranslation().getX() < Constants.FieldConstants.kRedAllianceLine) {
			if (driveSubsystem.getPose().getTranslation().getY() > Constants.FieldConstants.kCenterLine) {
				target = Constants.FieldConstants.flipPoseY(Constants.FieldConstants.kRedPassTargetRight);
			} else {
				target = Constants.FieldConstants.kRedPassTargetRight;
			}
		}

		shotCalculator.setTarget(target);
		turretSubsystem.moveToPosition(MathUtil.clamp(TurretConstants.kMin, TurretConstants.kMax,
				-1 * shotCalculator.turretYaw));

		shooterHoodSubsystem.moveToPosition(MathUtil.clamp(ShooterAngleConstants.kMax, ShooterAngleConstants.kMin,
			shotCalculator.hoodPitch));

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		turretSubsystem.stopMotor();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
