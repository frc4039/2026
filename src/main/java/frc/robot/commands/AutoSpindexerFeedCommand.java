package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;

public class AutoSpindexerFeedCommand extends Command{
    SpindexerSubsystem spindexerSubsystem;
	FeederSubsystem feederSubsystem;
	ShooterSubsystem shooterSubsystem;
	private boolean spindexerReverseMotor;
	private boolean feederReverseMotor;

	public AutoSpindexerFeedCommand(SpindexerSubsystem spindexerSubsystem, FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem, Boolean spindexerReverseMotor, Boolean feederReverseMotor) {
		addRequirements(feederSubsystem);
		this.spindexerSubsystem = spindexerSubsystem;
		this.feederSubsystem = feederSubsystem;
		this.spindexerReverseMotor = spindexerReverseMotor;
		this.feederReverseMotor = feederReverseMotor;
		this.shooterSubsystem = shooterSubsystem;
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		if (shooterSubsystem.atTargetSpeed() == false) {
			return;
		}
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
