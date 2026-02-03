package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterHoodSubsystem;
import frc.robot.subsystems.ShooterHoodSubsystem.ShooterAngleConstants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.TurretSubsystem.TurretConstants;

public class AutoAimCommand extends Command{
    private TurretSubsystem turretSubsystem;
	private DriveSubsystem driveSubsystem;
	private ShooterHoodSubsystem shooterHoodSubsystem;


	public AutoAimCommand(TurretSubsystem turretSubsystem, DriveSubsystem driveSubsystem,
			ShooterHoodSubsystem shooterHoodSubsystem) {
		this.turretSubsystem = turretSubsystem;
		this.driveSubsystem = driveSubsystem;
		this.shooterHoodSubsystem = shooterHoodSubsystem;
		addRequirements(turretSubsystem, shooterHoodSubsystem);

		
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		Pose2d currentRobotPose2d = driveSubsystem.getPose().plus(TurretConstants.kTurretOffset);
		Pose2d hubPose2d = TurretSubsystem.getHub();

    if(driveSubsystem.getPose().getTranslation().getX() < Constants.FieldConstants.kRedAllianceLine.getX()){
       if(driveSubsystem.getPose().getTranslation().getY() > Constants.FieldConstants.kCenterLine) {
        hubPose2d = Constants.FieldConstants.flipPoseY(Constants.FieldConstants.kRedPassTargetRight);
       } else {
          hubPose2d = Constants.FieldConstants.kRedPassTargetRight;
       }
    }


		turretSubsystem.moveToPosition(Math.min(TurretConstants.kMax, Math.max(TurretConstants.kMin,
				-1 * hubPose2d.relativeTo(currentRobotPose2d).getTranslation().getAngle().getDegrees())));

		shooterHoodSubsystem.moveToPosition((Math.min(ShooterAngleConstants.kMax,
				Math.max(ShooterAngleConstants.kMin, turretSubsystem.getHoodAngle()))));

		
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
