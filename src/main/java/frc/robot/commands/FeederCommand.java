package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SpindexerSubsystem;

public class FeederCommand extends Command {
	SpindexerSubsystem feederSubsystem;
	private boolean reverseMotor;

	public FeederCommand(SpindexerSubsystem feederSubsystem, Boolean reverseMotor) {
		addRequirements(feederSubsystem);
		this.feederSubsystem = feederSubsystem;
		this.reverseMotor = reverseMotor;
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		if (reverseMotor) {
			feederSubsystem.spin(true);
		} else {
			feederSubsystem.spin(false);
		}
	}

	@Override
	public void end(boolean interrupted) {
		feederSubsystem.stopMotor();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
