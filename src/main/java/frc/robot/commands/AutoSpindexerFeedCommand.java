package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AutoSpindexerFeedCommand extends Command{
    SpindexerSubsystem spindexerSubsystem;
	FeederSubsystem feederSubsystem;
	ShooterSubsystem shooterSubsystem;
	private boolean spindexerReverseMotor;
	private boolean feederReverseMotor;
	private TurretSubsystem turretSubsystem;
	public AutoSpindexerFeedCommand(SpindexerSubsystem spindexerSubsystem, FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem, Boolean spindexerReverseMotor, Boolean feederReverseMotor, TurretSubsystem turretSubsystem) {
		addRequirements(feederSubsystem);
		this.spindexerSubsystem = spindexerSubsystem;
		this.feederSubsystem = feederSubsystem;
		this.spindexerReverseMotor = spindexerReverseMotor;
		this.feederReverseMotor = feederReverseMotor;
		this.shooterSubsystem = shooterSubsystem;
		this.turretSubsystem = turretSubsystem;
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		if (shooterSubsystem.atTargetSpeed() == false) {
			spindexerSubsystem.stop();
			return;
		}
		if(!turretSubsystem.isAtPosition()) {
			spindexerSubsystem.stop();
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
		spindexerSubsystem.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
