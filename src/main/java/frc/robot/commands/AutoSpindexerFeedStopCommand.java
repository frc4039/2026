package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;

public class AutoSpindexerFeedStopCommand extends InstantCommand{
    SpindexerSubsystem spindexerSubsystem;
    FeederSubsystem feederSubsystem;
	public AutoSpindexerFeedStopCommand(SpindexerSubsystem spindexerSubsystem, FeederSubsystem feederSubsystem) {
		addRequirements(spindexerSubsystem,feederSubsystem);
		this.spindexerSubsystem = spindexerSubsystem;
        this.feederSubsystem = feederSubsystem;
	}

	@Override
	public void initialize() {
        spindexerSubsystem.stopMotor();
        feederSubsystem.stopMotor();
	}
}
