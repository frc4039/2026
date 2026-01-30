// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.TurretSubsystem.TurretConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimCommand extends Command {
  /** Creates a new AimCommand. */
  private TurretSubsystem turretSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private DriveSubsystem driveSubsystem;
  public AimCommand(TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem, DriveSubsystem driveSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.driveSubsystem = driveSubsystem;
    addRequirements(turretSubsystem, shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentRobotPose2d = driveSubsystem.getPose().plus(TurretConstants.kTurretOffset);
    Pose2d hubPose2d = TurretSubsystem.getHub();
    turretSubsystem.moveToPosition(Math.min(TurretConstants.kMax, Math.max(TurretConstants.kMin, -1 * hubPose2d.relativeTo(currentRobotPose2d).getTranslation().getAngle().getDegrees())));
    shooterSubsystem.shootInput(turretSubsystem.getOutputVelocity() / TurretConstants.kShooterWheelCircumference);
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
