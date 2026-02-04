// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.Console;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ObjectTrackingCommand extends Command {
  /** Creates a new ObjectTrackingCommand. */
  private VisionSubsystem visionSubsystem;
  private DriveSubsystem driveSubsystem;
  private ProfiledPIDController rotationController = new ProfiledPIDController(DriveConstants.kAimP, DriveConstants.kAimI, DriveConstants.kAimD, DriveConstants.kAimProfile);
  public ObjectTrackingCommand(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.visionSubsystem = visionSubsystem;
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationController.reset(Math.toRadians(driveSubsystem.getHeading()), 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetYaw = visionSubsystem.getTargetYaw();
    rotationController.reset(Math.toRadians(driveSubsystem.getHeading() - targetYaw));
    driveSubsystem.drive(0, 0, rotationController.calculate(Math.toRadians(driveSubsystem.getHeading())), true);
    System.out.println(targetYaw);
    // if(targetYaw != 0) {
    // if(targetYaw > 1) {
    //   driveSubsystem.drive(-0.5, 0, Units.degreesToRadians(10), false);
    // } if(targetYaw < -1) {
    //   driveSubsystem.drive(-0.5, 0, Units.degreesToRadians(-10), false);
    // } else {
    //   driveSubsystem.drive(-0.5, 0, 0, false);
    // }  
    // }
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
