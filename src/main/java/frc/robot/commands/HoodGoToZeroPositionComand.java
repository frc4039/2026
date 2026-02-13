// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterHoodSubsystem;
import frc.robot.subsystems.ShooterHoodSubsystem.ShooterAngleConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoodGoToZeroPositionComand extends Command {
  /** Creates a new HoodGoToZeroPositionComand. */
  private ShooterHoodSubsystem shooterHoodSubsystem;
  public HoodGoToZeroPositionComand(ShooterHoodSubsystem shooterHoodSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterHoodSubsystem = shooterHoodSubsystem;
    addRequirements(shooterHoodSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterHoodSubsystem.moveToPosition(ShooterAngleConstants.kHoodOffset / ShooterAngleConstants.kDegreesPerRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
