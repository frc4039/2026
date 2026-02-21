package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSlideSubsystem;
import frc.robot.subsystems.IntakeSlideSubsystem.IntakeSlideSubsystemConstants;

public class MoveIntakeSlideCommand extends Command {
	//Moves intake slide to specified position
	private final IntakeSlideSubsystem intakeSlideSubsystem;
	private double targetPosition;

	public MoveIntakeSlideCommand(IntakeSlideSubsystem intakeSlideSubsystem, double position) {
		this.intakeSlideSubsystem = intakeSlideSubsystem;

		this.targetPosition = position;

		addRequirements(intakeSlideSubsystem);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		intakeSlideSubsystem.moveToPosition(this.targetPosition);
	}

	@Override
	public void end(boolean interrupted) {
		intakeSlideSubsystem.stop();
	}

	@Override
	public boolean isFinished() {
		//Average position between left and right motor
		double positionAverage = (intakeSlideSubsystem.intakeSlideLeftMotor.getPosition().getValueAsDouble()
				+ intakeSlideSubsystem.intakeSlideRightMotor.getPosition().getValueAsDouble()) / 2;

		//Finished command if intake slide has moved within a reasonable threshold of error
		if (Math.abs(positionAverage - this.targetPosition) < IntakeSlideSubsystemConstants.kPositionThreshold) {
			return true;
		}

		return false;
	}
}
