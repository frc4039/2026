package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AutoSpindexerFeedCommand extends Command {
	SpindexerSubsystem spindexerSubsystem;
	FeederSubsystem feederSubsystem;
	ShooterSubsystem shooterSubsystem;
	private TurretSubsystem turretSubsystem;

	boolean startedSpinning = false;

	public AutoSpindexerFeedCommand(SpindexerSubsystem spindexerSubsystem, FeederSubsystem feederSubsystem,
			ShooterSubsystem shooterSubsystem, TurretSubsystem turretSubsystem) {
		addRequirements(feederSubsystem);
		this.spindexerSubsystem = spindexerSubsystem;
		this.feederSubsystem = feederSubsystem;
		this.shooterSubsystem = shooterSubsystem;
		this.turretSubsystem = turretSubsystem;
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		if(shooterSubsystem.atTargetSpeed() && turretSubsystem.isAtPosition()){
			spindexerSubsystem.spin(true);
			feederSubsystem.feed(true);
			startedSpinning = true;
		}
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return startedSpinning;
	}
}
