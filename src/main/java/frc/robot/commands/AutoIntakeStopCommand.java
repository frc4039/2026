package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeStopCommand extends Command {
	IntakeSubsystem intakeSubsystem;

	public AutoIntakeStopCommand(IntakeSubsystem intakeSubsystem) {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(intakeSubsystem);
		this.intakeSubsystem = intakeSubsystem;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		intakeSubsystem.stopMotor();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
	}
}
