package frc.robot.commands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AutoSpinUpCommand extends Command {
	private InterpolatingDoubleTreeMap shootingEstimator = new InterpolatingDoubleTreeMap();
	private final ShooterSubsystem shooterSubsystem;
	private final TurretSubsystem turretSubsystem;

	public AutoSpinUpCommand(ShooterSubsystem shooterSubsystem, TurretSubsystem turretSubsystem) {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(shooterSubsystem);
		this.shooterSubsystem = shooterSubsystem;
		this.turretSubsystem = turretSubsystem;
		shootingEstimator.put(7.517, 27.0);
		shootingEstimator.put(7.77, 31.0);
		shootingEstimator.put(8.24, 35.5);
		shootingEstimator.put(7.39, 24.0);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		double shootingSpeed = shootingEstimator.get(turretSubsystem.getOutputVelocity());
		shooterSubsystem.shootInput(shootingSpeed);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return shooterSubsystem.atTargetSpeed();
	}
}
