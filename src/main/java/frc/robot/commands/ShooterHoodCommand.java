package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterHoodSubsystem;

public class ShooterHoodCommand extends Command {
	private ShooterHoodSubsystem shooterHoodSubsystem;
	private double position;

	public ShooterHoodCommand(ShooterHoodSubsystem shooterHoodSub, double position) {
		this.shooterHoodSubsystem = shooterHoodSub;
		addRequirements(shooterHoodSubsystem);
		this.position = position;
	}

	@Override
	public void initialize() {
		System.out.println("In Shooter Hood Command");
		shooterHoodSubsystem.moveToPosition(position);
	}

	@Override
	public void execute() {
        System.out.println("Is executing");
	}

	@Override
	public void end(boolean interrupted) {
        System.out.println("Is ending");
	}

	@Override
	public boolean isFinished() {
        
		System.out.println(shooterHoodSubsystem.getHoodPosition() + " " + position);
		if (Math.abs(shooterHoodSubsystem.getHoodPosition()
				- position) < ShooterHoodSubsystem.ShooterAngleConstants.angleThreshold) {
			return true;
		}
        
		return false;
	}
}

