package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;

public class AutoSpindexerFeedCommand extends Command{
    SpindexerSubsystem spindexerSubsystem;
	FeederSubsystem feederSubsystem;
	private boolean spindexerReverseMotor;
	private boolean feederReverseMotor;

	public AutoSpindexerFeedCommand(SpindexerSubsystem spindexerSubsystem, FeederSubsystem feederSubsystem, Boolean spindexerReverseMotor, Boolean feederReverseMotor) {
		addRequirements(feederSubsystem);
		this.spindexerSubsystem = spindexerSubsystem;
		this.feederSubsystem = feederSubsystem;
		this.spindexerReverseMotor = spindexerReverseMotor;
		this.feederReverseMotor = feederReverseMotor;
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		if (spindexerReverseMotor) {
			spindexerSubsystem.spin(true);
		} else {
			spindexerSubsystem.spin(false);
		}

		if(feederReverseMotor){
			feederSubsystem.feed(true);
		} else {
			feederSubsystem.feed(false);
		}
	}

	@Override
	public void end(boolean interrupted) {
		spindexerSubsystem.stopMotor();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
