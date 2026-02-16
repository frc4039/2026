package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSlideSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSlideSubsystem.IntakeSlideSubsystemConstants;

public class IntakeOutCommand extends SequentialCommandGroup {
	public IntakeOutCommand(IntakeSubsystem intakeSubsystem, IntakeSlideSubsystem intakeSlideSubsystem) {
		addCommands(new MoveIntakeSlideCommand(intakeSlideSubsystem, IntakeSlideSubsystemConstants.kOutPosition),
				new IntakeCommand(intakeSubsystem, true));
	}
}
