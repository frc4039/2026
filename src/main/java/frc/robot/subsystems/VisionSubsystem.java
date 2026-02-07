
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.TurretSubsystem.TurretConstants;
import frc.robot.utils.HardwareMonitor;
import frc.robot.utils.Helpers;
import frc.robot.utils.ShotCalculator;

public class VisionSubsystem extends SubsystemBase {
    /** Creates a new VisionSubsystem. */

  public static class VisionConstants {
    //public static final String kFrontRightCameraName = "LimelightFrontRight";
    public static final String kFrontLeftCameraName = "FrontLeft-Camera";
    public static final String kFrontRightCameraName = "FrontRight-Camera";
   // public static final String kBackCameraName = "LimelightBack";
    public static final double xOffset = Helpers.isBabycakes() ?0.42 + .105: 0.42 + .11; // 0.42m is the offest from the center of the robot to the edge of the bumper.  
    public static final double yOffsetRight = 0.164;  //  Distance from the center of the tag to the center of the branch right.
    public static final double yOffsetLeft = -0.164; 
    public static final double xOffsetL1 = 0.462;
    public static final double yOffsetRightL1 = 0.170;
    public static final double yOffsetLeftL1 = -0.170;
    public static final double rightAngleOffset = -22.17;
    public static final double leftAngleOffset = 22.17;
    
    public static final Pose2d redTower = new Pose2d(15.44, 5.6, new Rotation2d(Units.degreesToRadians(0)));
    public static final Pose2d blueTower = new Pose2d(1, 1, new Rotation2d(1));

    public static final Pose2d KFinalRedTower = new Pose2d(15.44, 5.37, new Rotation2d(Units.degreesToRadians(0)));


    //  Array of Tag specific Offsets, For quick refernce Array index is Tag ID
    //  Values are robot centric, x + is away from the reef, y + is right
    //  First Pair is the Left offset, second pair is the right.

    public static final double[][] tagSpecificOffset = {{0,0,0,0},  // Tag ID 0
                                                        {0,0,0,0},  // Tag ID 1
                                                        {0,0,0,0},  // Tag ID 2
                                                        {0,0,0,0},  // Tag ID 3
                                                        {0,0,0,0},  // Tag ID 4
                                                        {0,0,0,0},  // Tag ID 5
                                                        {0,0,0,0},  // Tag ID 6
                                                        {0,0,0,0},  // Tag ID 7
                                                        {0,0,0,0},  // Tag ID 8
                                                        {0,0,0,0},  // Tag ID 9
                                                        {0,0,0,0},  // Tag ID 10
                                                        {0,0,0,0},  // Tag ID 11
                                                        {0,0,0,0},  // Tag ID 12
                                                        {0,0,0,0},  // Tag ID 13
                                                        {0,0,0,0},  // Tag ID 14
                                                        {0,0,0,0},  // Tag ID 15
                                                        {0,0,0,0},  // Tag ID 16
                                                        {0,0,0,0},  // Tag ID 17
                                                        {0,0,0,0},  // Tag ID 18
                                                        {0,0,0,0},  // Tag ID 19
                                                        {0,0,0,0},  // Tag ID 20
                                                        {0,0,0,0},  // Tag ID 21
                                                        {0,0,0,0}   // Tag ID 22
                                                      };  
    
    public static final double AngTolerance = 25.0 / 180 * Math.PI;

    // Cam mounted facing forward, half a meter forward of center, half a meter up
    // from center.
    public static final Transform3d kRobotToCamFrontRight =
        new Transform3d(
          new Translation3d(Units.inchesToMeters(12), Units.inchesToMeters(-11.25), Units.inchesToMeters(20)),
          new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-24.5),
          Units.degreesToRadians(-13.5)));

    public static final Transform3d kRobotToCamFrontLeft =
        new Transform3d(
          new Translation3d(Units.inchesToMeters(11 + (1/16)), Units.inchesToMeters(13), Units.inchesToMeters(16 + (5/6))),
          new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(90)));
        
  
/*     public static final Transform3d kRobotToCamBack = 
      Helpers.isBabycakes()?
        new Transform3d(
          new Translation3d(-0.013, 0.26, 0.856),
          new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-20),
          Units.degreesToRadians(180))) :
        new Transform3d(
          new Translation3d(-0.013, 0.26,0.856),
          new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-20),
          Units.degreesToRadians(180)));
*/    
    public static final double kMaxGyroCameraAngleDelta = 89.0;

    }

    public class VisionTargetTag {
      public int tagID;
      public Pose2d pose;
    }

    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final PhotonCamera frontRightCam;
    private final PhotonCamera frontLeftCam;
//    private final PhotonCamera backCam;
    private final DriveSubsystem m_driveSubsystem;
    private final TurretSubsystem turretSubsystem;
    private Field2d fieldDisplay = new Field2d();
    private List<Pose2d> blueTargets = new ArrayList<>();
    private List<Pose2d> redTargets = new ArrayList<>();
    private VisionTargetTag target = new VisionTargetTag();
    private List<Pose2d> blueCages = new ArrayList<>();
    private List<Pose2d> redCages = new ArrayList<>();

    // TODO add new poses for blue and red cages                                                  // Theretical Values  //Robodrome Values
    private Pose2d blueCage1 = new Pose2d(8.05, 7.23, Rotation2d.fromDegrees(180));   // x:8.29, y:7.23
    private Pose2d blueCage2 = new Pose2d(8.05, 6.04, Rotation2d.fromDegrees(180));   // x:8.29, y:6.14     // Robodrome x:8.17, y:6.14
    private Pose2d blueCage3 = new Pose2d(8.05, 5.01, Rotation2d.fromDegrees(180));   // x:8.29, y:5.05     // Robodrome x:8.17. y:4.98
    private Pose2d redCage1 = new Pose2d(9.45, 0.75, Rotation2d.fromDegrees(0));      // x:9.26, y:0.83
    private Pose2d redCage2 = new Pose2d(9.45, 1.93, Rotation2d.fromDegrees(0));      // x:9.26, y:1.92
    private Pose2d redCage3 = new Pose2d(9.45, 2.95, Rotation2d.fromDegrees(0));  

    // Construct PhotonPoseEstimator
   public final PhotonPoseEstimator frontRightPhotonPoseEstimator;
    public final PhotonPoseEstimator frontLeftPhotonPoseEstimator;
//    public final PhotonPoseEstimator backPhotonPoseEstimator;
    private final ShotCalculator shotCalculator;

    public VisionSubsystem(DriveSubsystem driveSubsystem, TurretSubsystem turretSubsystem, ShotCalculator shotCalculator) {
        m_driveSubsystem = driveSubsystem;
        this.turretSubsystem = turretSubsystem;
        this.shotCalculator = shotCalculator;
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    frontRightCam = new PhotonCamera(VisionConstants.kFrontRightCameraName);
    frontLeftCam = new PhotonCamera(VisionConstants.kFrontLeftCameraName);
//    backCam = new PhotonCamera(VisionConstants.kBackCameraName);
    // hw.registerDevice(this, frontRightCam);
    // hw.registerDevice(this, frontLeftCam);
//    hw.registerDevice(this, backCam);
    
    frontRightPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRobotToCamFrontRight);
    frontRightPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

    frontLeftPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRobotToCamFrontLeft);
    frontLeftPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

/*      backPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRobotToCamBack);
    backPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
*/
        SmartDashboard.putData("Field", fieldDisplay);

        PathPlannerLogging.setLogTargetPoseCallback((target) -> {
            fieldDisplay.getObject("target pose").setPose(target);
        });
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            fieldDisplay.getObject("path").setPoses(poses);
        });
        
        blueTargets.add(getTagPose(17).toPose2d());
        blueTargets.add(getTagPose(18).toPose2d());
        blueTargets.add(getTagPose(19).toPose2d());
        blueTargets.add(getTagPose(20).toPose2d());
        blueTargets.add(getTagPose(21).toPose2d());
        blueTargets.add(getTagPose(22).toPose2d());
        redTargets.add(getTagPose(6).toPose2d());
        redTargets.add(getTagPose(7).toPose2d());
        redTargets.add(getTagPose(8).toPose2d());
        redTargets.add(getTagPose(9).toPose2d());
        redTargets.add(getTagPose(10).toPose2d());
        redTargets.add(getTagPose(11).toPose2d());

        blueCages.add(blueCage1);
        blueCages.add(blueCage2);
        blueCages.add(blueCage3);

        redCages.add(redCage1);
        redCages.add(redCage2);
        redCages.add(redCage3);

        
    }

    /**
     * Update the pose estimator with the pending results from a single camera.
     */
    private void updateCamera(PhotonCamera cam, PhotonPoseEstimator estimator, String poseName) {
        final List<PhotonPipelineResult> results = cam.getAllUnreadResults();

        if (results.size() > 0) {
            var lastResult = results.get(results.size() - 1);
            Optional<EstimatedRobotPose> estimatedPose = estimator.update(lastResult);
            if (estimatedPose.isPresent()) {
                m_driveSubsystem.addVisionPose(estimatedPose.get().estimatedPose.toPose2d(),
                        estimatedPose.get().timestampSeconds,
                        poseName);
            }

            if (estimatedPose.isPresent()) {
                fieldDisplay.getObject(poseName).setPose(estimatedPose.get().estimatedPose.toPose2d());
            } else {
                fieldDisplay.getObject(poseName).setPose(new Pose2d(-100.0, -100.0, new Rotation2d()));
            }
        }
  }
  public Pose2d getBlueTowerPose() {
    return VisionSubsystem.VisionConstants.blueTower;
    // return  m_driveSubsystem.getPose().nearest(VisionSubsystem.VisionConstants.blueTower);
  }

  public Pose2d getRedTowerPose() {
    return VisionSubsystem.VisionConstants.redTower;
    // return  m_driveSubsystem.getPose().nearest(VisionSubsystem.VisionConstants.redTower);
  }

  public Pose2d getClosestReefTargetRed() {
    double RobotRearOrientation = m_driveSubsystem.getPose().getRotation().getRadians() - Math.PI;
    target.tagID = 0;

    for (int i = 0; i< redTargets.size();i++){
      if (Math.cos(redTargets.get(i).getRotation().getRadians() - RobotRearOrientation)> Math.cos(VisionConstants.AngTolerance)){
        target.pose = redTargets.get(i);
        target.tagID = i + 6;
        return target.pose;
      }
    }
    target.pose = m_driveSubsystem.getPose().nearest(redTargets);
    for (int i = 0; i< redTargets.size();i++){
      if (target.pose.getX() == redTargets.get(i).getX() && target.pose.getY() == redTargets.get(i).getY())
        target.tagID = i + 6;
    }  
    return target.pose;
  }

  public Pose2d getClosestReefTargetBlue() {
    double RobotRearOrientation = m_driveSubsystem.getPose().getRotation().getRadians() - Math.PI;
    target.tagID = 0;
    
    for (int i = 0; i< blueTargets.size();i++){
      if (Math.cos(blueTargets.get(i).getRotation().getRadians() - RobotRearOrientation)> Math.cos(VisionConstants.AngTolerance)){
        target.pose = blueTargets.get(i);
        target.tagID = i + 17;
        return target.pose;
      }
    }
    target.pose = m_driveSubsystem.getPose().nearest(blueTargets);
    for (int i = 0; i< blueTargets.size();i++){
      if (target.pose.getX() == blueTargets.get(i).getX() && target.pose.getY() == blueTargets.get(i).getY())
        target.tagID = i + 17;
    }  
    return target.pose;

  }

  public int getTargetTagID() {
    return target.tagID;
    }

  public Pose3d getTagPose(int tagID) {
    return aprilTagFieldLayout.getTagPose(tagID).get();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        frontRightPhotonPoseEstimator.setReferencePose(m_driveSubsystem.getPose());
        updateCamera(frontRightCam, frontRightPhotonPoseEstimator, "RightFrontPose");
        
        frontLeftPhotonPoseEstimator.setReferencePose(m_driveSubsystem.getPose());
        updateCamera(frontLeftCam, frontLeftPhotonPoseEstimator, "LeftFrontPose");
        
//        backPhotonPoseEstimator.setReferencePose(m_driveSubsystem.getPose());
//        updateCamera(backCam, backPhotonPoseEstimator, "BackPose");

        fieldDisplay.setRobotPose(m_driveSubsystem.getPose());
        fieldDisplay.getObject("target").setPose(shotCalculator.getTargetPose());
        //fieldDisplay.getObject("turret").setPose(turretSubsystem.getTurretPose(m_driveSubsystem.getPose()));

        var turretPose = new Pose3d(m_driveSubsystem.getPose()).plus(TurretConstants.kTurretOffset).toPose2d().plus(new Transform2d( new Translation2d(), Rotation2d.fromDegrees(-1 * shotCalculator.turretYaw)));
        fieldDisplay.getObject("Turret").setPose(turretPose);

    }
}
