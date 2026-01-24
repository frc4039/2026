package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;

public class TurretFeederCommand extends Command {
	FeederSubsystem turretFeederSubsystem;
	private boolean reverseMotor;

	public TurretFeederCommand(FeederSubsystem turretFeederSubsystem, Boolean reverseMotor) {
		addRequirements(turretFeederSubsystem);
		this.turretFeederSubsystem = turretFeederSubsystem;
		this.reverseMotor = reverseMotor;
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		if (reverseMotor) {
			turretFeederSubsystem.feed(true);
		} else {
			turretFeederSubsystem.feed(false);
		}
	}

	@Override
	public void end(boolean interrupted) {
		turretFeederSubsystem.stopMotor();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
