package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class SpindexerCommand extends Command {
	SpindexerSubsystem feederSubsystem;
	private TurretSubsystem turretSubsystem;
	private ShooterSubsystem shooterSubsystem;
	private boolean reverseMotor;

	public SpindexerCommand(TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem, SpindexerSubsystem feederSubsystem, Boolean reverseMotor) {
		addRequirements(feederSubsystem);
		this.feederSubsystem = feederSubsystem;
		this.reverseMotor = reverseMotor;
		this.turretSubsystem = turretSubsystem;
		this.shooterSubsystem = shooterSubsystem;
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		if(Math.abs(shooterSubsystem.getShooterError()) < 5 && Math.abs(turretSubsystem.getTurretError()) < 5) {
		if (reverseMotor) {
			feederSubsystem.spin(false);
		} else {
			feederSubsystem.spin(true);
		}
		}else {
			feederSubsystem.stop();
		}
	}

	@Override
	public void end(boolean interrupted) {
		feederSubsystem.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
