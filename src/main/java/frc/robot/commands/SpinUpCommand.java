// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.TurretSubsystem.TurretConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinUpCommand extends Command {
  /** Creates a new SpinUpCommand. */
  private InterpolatingDoubleTreeMap shootingEstimator = new InterpolatingDoubleTreeMap();
  private final ShooterSubsystem shooterSubsystem;
  private final TurretSubsystem turretSubsystem;
  private double multiplier;
  public SpinUpCommand(ShooterSubsystem shooterSubsystem, TurretSubsystem turretSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem; 
    this.turretSubsystem = turretSubsystem;
   

    shootingEstimator.put(4.208, 15.0);
		shootingEstimator.put(4.757, 17.0);
		shootingEstimator.put(5.488, 19.5);
		shootingEstimator.put(5.987, 21.0);
    shootingEstimator.put(6.555, 23.0);
    shootingEstimator.put(7.016, 25.0);
    shootingEstimator.put(7.362, 27.0);
    shootingEstimator.put(7.874, 29.0);
    shootingEstimator.put(8.346, 31.0);
    shootingEstimator.put(8.677, 33.0);
    shootingEstimator.put(8.728, 35.0);
    shootingEstimator.put(9.101, 37.0);
    shootingEstimator.put(9.405, 39.0);
    shootingEstimator.put(9.779, 41.0);
    shootingEstimator.put(10.295, 43.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //this should lower the velocity when close to the hub to account for the fact that the hood cant get low enough, CloseShotThreshold is 0 for now
    if(turretSubsystem.getOutputVelocity() < TurretConstants.kCloseShotThreshold) {
      multiplier = 0.95;
    }
    else if(turretSubsystem.getOutputVelocity() > 8) {
      multiplier = 1.03;
    } else {
      multiplier = 1.0;
    }

    double shootingSpeed = (shootingEstimator.get(turretSubsystem.getOutputVelocity())) * multiplier;
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
