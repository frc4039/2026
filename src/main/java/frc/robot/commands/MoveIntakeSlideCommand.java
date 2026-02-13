// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSlideSubsystem;
import frc.robot.subsystems.IntakeSlideSubsystem.IntakeSlideSubsystemConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveIntakeSlideCommand extends Command {
	/** Creates a new MoveIntakeSlideCommand. */
	private final IntakeSlideSubsystem intakeSlideSubsystem;
	private double targetPosition;

	public MoveIntakeSlideCommand(IntakeSlideSubsystem intakeSlideSubsystem, boolean in) {
		this.intakeSlideSubsystem = intakeSlideSubsystem;

		if (in) {
			this.targetPosition = IntakeSlideSubsystemConstants.kInPosition;
		} else {
			this.targetPosition = IntakeSlideSubsystemConstants.kOutPosition;
		}

		addRequirements(intakeSlideSubsystem);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		intakeSlideSubsystem.moveToPosition(this.targetPosition);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		intakeSlideSubsystem.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		double positionAverage = (intakeSlideSubsystem.intakeSlideLeftMotor.getPosition().getValueAsDouble()
				+ intakeSlideSubsystem.intakeSlideRightMotor.getPosition().getValueAsDouble()) / 2;

		if (Math.abs(positionAverage - this.targetPosition) < IntakeSlideSubsystemConstants.kPositionThreshold) {
			return true;
		}

		return false;
	}
}
