// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionConstants;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToTowerCommand extends DeferredCommand {

  public AlignToTowerCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    super(() -> getPathCommand(driveSubsystem, visionSubsystem),Set.of(driveSubsystem));
  }


  // Called when the command is initially scheduled.
  public static Command getPathCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
  Pose2d Target = VisionConstants.redTower;
  double endX;
  double endY;
  
  var alliance = DriverStation.getAlliance();
  // if (alliance.isPresent()) {
  //     if (alliance.get() == DriverStation.Alliance.Red){
  //       Target = visionSubsystem.getRedTowerPose();
  //     }
  //     else{
  //       Target = visionSubsystem.getBlueTowerPose();
  //     }
  // }else{
  //   return null;
  // }

    endX = Target.getX();// + finalXOffset * Math.cos(Target.getRotation().getRadians()) - finalYOffset * Math.sin(Target.getRotation().getRadians());
    endY = Target.getY();// + finalXOffset * Math.sin(Target.getRotation().getRadians()) + finalYOffset * Math.cos(Target.getRotation().getRadians());
  

  double startX = driveSubsystem.getPoseXValue();
  double startY = driveSubsystem.getPoseYValue();

  double x = endX - startX;
  double y = endY - startY;

  if ( (x*x + y*y) > (0.02*0.02) ){ // Only drive to path when total distance to target is greater than .02m
    double endAngle = Math.atan2(y,x);
    double startAngle = driveSubsystem.getSpeed() > 0.5 ? driveSubsystem.getRobotVectorAngle() 
                                                        : endAngle;

    Pose2d start = new Pose2d(startX, startY, Rotation2d.fromRadians(startAngle));
    Pose2d end = new Pose2d(endX, endY, Rotation2d.fromRadians(endAngle));
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses( start, end);

    PathConstraints constraints = new PathConstraints(0.5, 1.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
      PathPlannerPath path = new PathPlannerPath(
          waypoints,
          constraints,
          null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
          new GoalEndState(0.0, Rotation2d.fromDegrees((Target.getRotation().getDegrees())))); // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    path.preventFlipping = true;

    return AutoBuilder.followPath(path).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }
    return null;
  }
}