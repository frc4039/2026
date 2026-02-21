package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class FeederCommand extends Command {
	FeederSubsystem turretFeederSubsystem;
	private boolean reverseMotor;
	private ShooterSubsystem shooterSubsystem;
	private TurretSubsystem turretSubsystem;

	public FeederCommand(ShooterSubsystem shooterSubsystem, TurretSubsystem turretSubsystem,
			FeederSubsystem turretFeederSubsystem, Boolean reverseMotor) {
		addRequirements(turretFeederSubsystem);
		this.turretFeederSubsystem = turretFeederSubsystem;
		this.reverseMotor = reverseMotor;
		this.turretSubsystem = turretSubsystem;
		this.shooterSubsystem = shooterSubsystem;
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		// Shoot if the turret is at the right angle and the shooter is at the right
		// speed.
		if (Math.abs(shooterSubsystem.getShooterError()) < 5 && Math.abs(turretSubsystem.getTurretError()) < 5) {
			if (reverseMotor) {
				turretFeederSubsystem.feed(true);
			} else {
				turretFeederSubsystem.feed(false);
			}
		} else {
			turretFeederSubsystem.stop();
		}
	}

	@Override
	public void end(boolean interrupted) {
		turretFeederSubsystem.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
