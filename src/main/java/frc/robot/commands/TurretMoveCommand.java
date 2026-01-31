package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.TurretSubsystem;

public class TurretMoveCommand extends Command {
	private TurretSubsystem turretSubsystem;
	private double position;

	public TurretMoveCommand(TurretSubsystem turretSubsystem, double position) {
		this.turretSubsystem = turretSubsystem;
		addRequirements(turretSubsystem);
		this.position = position;
	}

	@Override
	public void initialize() {
		System.out.println("In Turret Move Command");
		turretSubsystem.moveToPosition(position);
	}

	@Override
	public void execute() {
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		if (Math.abs(turretSubsystem.getTurretPosition()
				- position) < TurretSubsystem.TurretConstants.kTurretAccuracyThreshold) {
			return true;
		}

		return false;
	}
}
