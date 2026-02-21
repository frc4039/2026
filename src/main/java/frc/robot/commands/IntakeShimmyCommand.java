package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSlideSubsystem;
import frc.robot.subsystems.IntakeSlideSubsystem.IntakeSlideSubsystemConstants;

public class IntakeShimmyCommand extends SequentialCommandGroup {
	public IntakeShimmyCommand(IntakeSlideSubsystem intakeSlideSubsystem) {
		// Move the intake in and then out a random amount.
		addCommands(new MoveIntakeSlideCommand(intakeSlideSubsystem, IntakeSlideSubsystemConstants.kOutPosition),
				new MoveIntakeSlideCommand(intakeSlideSubsystem,
						IntakeSlideSubsystemConstants.kOutPosition + 5 + Math.floor(Math.random() * 5)));
	}
}
