package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeStopCommand extends Command {
	IntakeSubsystem intakeSubsystem;
	private boolean intake;

	public AutoIntakeCommand(IntakeSubsystem intakeSubsystem, Boolean intake) {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(intakeSubsystem);
		this.intakeSubsystem = intakeSubsystem;
		this.intake = intake;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (intake) {
			intakeSubsystem.intake();
		} else {
			intakeSubsystem.outtake();
		}
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
