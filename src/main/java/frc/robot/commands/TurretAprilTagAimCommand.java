// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.TurretSubsystem.TurretConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretAprilTagAimCommand extends Command {
  /** Creates a new TurretWithJoystickCommand. */
  private final TurretSubsystem turretSubsystem;
  private final DriveSubsystem driveSubsystem;

  public TurretAprilTagAimCommand(TurretSubsystem turretSubsystem, DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turretSubsystem);
    this.turretSubsystem = turretSubsystem;
    this.driveSubsystem = driveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Pose2d currentRobotPose2d = driveSubsystem.getPose().plus(TurretConstants.kTurretOffset);
    // Pose2d hubPose2d = TurretSubsystem.getHub();
    // SmartDashboard.putData("Before Clamp", new Sendable() {
    //   @Override
    //   public void initSendable(SendableBuilder builder) {
    //     builder.addDoubleProperty("Before Clamp", () -> -1 * hubPose2d.relativeTo(currentRobotPose2d).getTranslation().getAngle().getDegrees(), null);
    //   }
    // });
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentRobotPose2d = driveSubsystem.getPose().plus(TurretConstants.kTurretOffset);
    Pose2d hubPose2d = TurretSubsystem.getHub();
    turretSubsystem.moveToPosition(-1 * hubPose2d.relativeTo(currentRobotPose2d).getTranslation().getAngle().getDegrees());
    //turretSubsystem.moveToPosition(Math.min(TurretConstants.kMax, Math.max(TurretConstants.kMin, -1 * hubPose2d.relativeTo(currentRobotPose2d).getTranslation().getAngle().getDegrees())));
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
