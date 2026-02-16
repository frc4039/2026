// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterHoodSubsystem;
import frc.robot.subsystems.ShooterHoodSubsystem.ShooterAngleConstants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.TurretSubsystem.AimState;
import frc.robot.subsystems.TurretSubsystem.TurretConstants;
import frc.robot.utils.ShotCalculator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimCommand extends Command {
	/** Creates a new AimCommand. */
	private TurretSubsystem turretSubsystem;
	private DriveSubsystem driveSubsystem;
	private ShooterHoodSubsystem shooterHoodSubsystem;
	private Supplier<AimState> currentAim;
	private ShotCalculator shotCalculator;

	public AimCommand(TurretSubsystem turretSubsystem, DriveSubsystem driveSubsystem,
			ShooterHoodSubsystem shooterHoodSubsystem, ShotCalculator shotCalculator, Supplier<AimState> currentAim) {
		this.turretSubsystem = turretSubsystem;
		this.driveSubsystem = driveSubsystem;
		this.shooterHoodSubsystem = shooterHoodSubsystem;
		this.shotCalculator = shotCalculator;
		this.currentAim = currentAim;
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

		Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

		double driveX = driveSubsystem.getPose().getTranslation().getX();

		if(driveX < Constants.FieldConstants.kRedAllianceLine.getX() && driveX > Constants.FieldConstants.kBlueAllianceLine.getX()){
			// Change target for shuttling
			if(alliance == Alliance.Red){
				target = Constants.FieldConstants.kRedPassTargetRight;
			}else{
				target = Constants.FieldConstants.kBluePassTargetRight;
			}

			// Adjust target to left or right side
			if((driveSubsystem.getPose().getTranslation().getY() < Constants.FieldConstants.kCenterLine && currentAim.get().equals(AimState.AUTOMATIC)) || currentAim.get().equals(AimState.LEFT)){
				target = Constants.FieldConstants.flipPoseY(target);
			}
		}

		shotCalculator.setTarget(target);

		turretSubsystem.moveToPosition(MathUtil.clamp( shotCalculator.turretYaw, TurretConstants.kMin,
				TurretConstants.kMax));

		if ((driveSubsystem.getPose().getTranslation().getX() > Units.inchesToMeters(157)
				&& driveSubsystem.getPose().getTranslation().getX() < Units.inchesToMeters(205))
				|| (driveSubsystem.getPose().getTranslation().getX() > Units.inchesToMeters(444)
						&& driveSubsystem.getPose().getTranslation().getX() < Units.inchesToMeters(492))) {
			shooterHoodSubsystem.moveToPosition(ShooterAngleConstants.kMax);

		} else {
		shooterHoodSubsystem.moveToPosition(MathUtil.clamp(ShooterAngleConstants.kMax, ShooterAngleConstants.kMin,
			shotCalculator.hoodPitch));
		}
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
