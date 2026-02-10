// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinUpCommand extends Command {
  /** Creates a new SpinUpCommand. */
  private InterpolatingDoubleTreeMap shootingEstimator = new InterpolatingDoubleTreeMap();
  private final ShooterSubsystem shooterSubsystem;
  private final TurretSubsystem turretSubsystem;
  public SpinUpCommand(ShooterSubsystem shooterSubsystem, TurretSubsystem turretSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem; 
    this.turretSubsystem = turretSubsystem;
    shootingEstimator.put(7.517, 27.0);
		shootingEstimator.put(7.77, 31.0);
		shootingEstimator.put(8.24, 35.5);
		shootingEstimator.put(7.39, 24.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shootingSpeed = shootingEstimator.get(turretSubsystem.getOutputVelocity());
		shooterSubsystem.shootInput(shootingSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
