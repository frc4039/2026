// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.ShotCalculator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinUpCommand extends Command {
  /** Creates a new SpinUpCommand. */
  private final ShooterSubsystem shooterSubsystem;
  private final ShotCalculator shotCalculator;

  public SpinUpCommand(ShooterSubsystem shooterSubsystem, ShotCalculator shotCalculator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem; 
    this.shotCalculator = shotCalculator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
		shooterSubsystem.shootInput(shotCalculator.shooterRps);
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
