// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorPort = 1;
  }

  public static class FieldConstants {
    public static final double kFieldLength = 651.22;
    public static final double kDistanceFromHubToWall = 182.11;
    public static final Pose2d kBlueHub = new Pose2d(Units.inchesToMeters(182.11), Units.inchesToMeters(159), new Rotation2d(0));
    public static final Pose2d kRedHub = new Pose2d(Units.inchesToMeters(469.11), Units.inchesToMeters(159), new Rotation2d(0));
    public static final Pose2d kRedAllianceLine = new Pose2d(Units.inchesToMeters(469.11), 0, new Rotation2d(0));
    public static final Pose2d kRedPassTargetRight = new Pose2d(Units.inchesToMeters(560), Units.inchesToMeters(75), new Rotation2d(0));
    public static final Pose2d kLeftPassTarget = new Pose2d(Units.inchesToMeters(560), Units.inchesToMeters(240), new Rotation2d(0));
    public static final double kCenterLine = Units.inchesToMeters(158.32);
    public static Pose2d flipPoseY(Pose2d yPose2d) {
      return new Pose2d(yPose2d.getX(), Units.inchesToMeters(316.64) - yPose2d.getY(), yPose2d.getRotation());
    }
    public static Pose2d flipPoseX(Pose2d xPose2d) {
      return new Pose2d(Units.inchesToMeters(650.12) - xPose2d.getX(), xPose2d.getY(), xPose2d.getRotation());
    }
  }
}
