package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndgameSubsystem;

public class ClimbCommand extends Command {
	EndgameSubsystem endgameSubsystem;
	private boolean goUpwards;

	public ClimbCommand(EndgameSubsystem endgameSubsystem, boolean goUpwards) {
		addRequirements(endgameSubsystem);

		this.endgameSubsystem = endgameSubsystem;
		this.goUpwards = goUpwards;
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		if (goUpwards) {
			endgameSubsystem.climb(true);
		} else {
			endgameSubsystem.climb(false);
		}
	}

	@Override
	public void end(boolean interrupted) {
		endgameSubsystem.stopMotor();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
