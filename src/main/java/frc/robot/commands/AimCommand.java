package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
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

public class AimCommand extends Command {
	private TurretSubsystem turretSubsystem;
	private DriveSubsystem driveSubsystem;
	private ShooterHoodSubsystem shooterHoodSubsystem;

	private Supplier<AimState> currentAim;

	public AimCommand(TurretSubsystem turretSubsystem, DriveSubsystem driveSubsystem,
			ShooterHoodSubsystem shooterHoodSubsystem, Supplier<AimState> currentAim) {
		this.turretSubsystem = turretSubsystem;
		this.driveSubsystem = driveSubsystem;
		this.shooterHoodSubsystem = shooterHoodSubsystem;
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
		Pose2d currentRobotPose2d = driveSubsystem.getShootOnTheFlyPose2d().plus(TurretConstants.kTurretOffset);
		Pose2d hubPose2d = TurretSubsystem.getHub();

		// Get the current alliance
		Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

		// Current x position on the field.
		double driveX = driveSubsystem.getPose().getTranslation().getX();

		// Check if in neutral zone for shuttling.
		// TODO: check for neutral zone and opposite aliance zone.
		if (driveX < Constants.FieldConstants.kRedAllianceLine.getX()
				&& driveX > Constants.FieldConstants.kBlueAllianceLine.getX()) {
			// Change target for shuttling
			if (alliance == Alliance.Red) {
				hubPose2d = Constants.FieldConstants.kRedPassTargetRight;
			} else {
				hubPose2d = Constants.FieldConstants.kBluePassTargetRight;
			}

			// Adjust target to left or right side
			if ((driveSubsystem.getPose().getTranslation().getY() < Constants.FieldConstants.kCenterLine
					&& currentAim.get().equals(AimState.AUTOMATIC)) || currentAim.get().equals(AimState.LEFT)) {
				hubPose2d = Constants.FieldConstants.flipPoseY(hubPose2d);
			}
		}

		// Move the turret to the calculated position.
		turretSubsystem.moveToPosition(Math.min(TurretConstants.kMax, Math.max(TurretConstants.kMin,
				-1 * hubPose2d.relativeTo(currentRobotPose2d).getTranslation().getAngle().getDegrees())));

		// Lower the hood if near the trench.
		if ((driveSubsystem.getPose().getTranslation().getX() > Units.inchesToMeters(157)
				&& driveSubsystem.getPose().getTranslation().getX() < Units.inchesToMeters(205))
				|| (driveSubsystem.getPose().getTranslation().getX() > Units.inchesToMeters(444)
						&& driveSubsystem.getPose().getTranslation().getX() < Units.inchesToMeters(492))) {
			shooterHoodSubsystem.moveToPosition((Math.min(ShooterAngleConstants.kMax,
					Math.max(ShooterAngleConstants.kMin, ShooterAngleConstants.kMax))));

		} else {
			// Otherwise move the hood to the calculated position.
			shooterHoodSubsystem.moveToPosition((Math.min(ShooterAngleConstants.kMax,
					Math.max(ShooterAngleConstants.kMin, turretSubsystem.getHoodAngle()))));
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
