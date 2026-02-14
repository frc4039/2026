package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSlideSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOutCommand extends SequentialCommandGroup {
	public IntakeOutCommand(IntakeSubsystem intakeSubsystem, IntakeSlideSubsystem intakeSlideSubsystem) {
		addCommands(new MoveIntakeSlideCommand(intakeSlideSubsystem, false),
				new IntakeCommand(intakeSubsystem, true));
	}
}
