
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
import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.HardwareMonitor;

public class VisionSubsystem extends SubsystemBase {
	/** Creates a new VisionSubsystem. */
	public static class VisionConstants {
		public static final String kFrontLeftCameraName = "FrontLeft-Camera";
		public static final String kFrontRightCameraName = "FrontRight-Camera";

		public static final Pose2d redTower = new Pose2d(15.44, 5.6, new Rotation2d(Units.degreesToRadians(0)));
		public static final Pose2d blueTower = new Pose2d(1, 1, new Rotation2d(1));

		public static final Pose2d KFinalRedTower = new Pose2d(15.44, 5.37, new Rotation2d(Units.degreesToRadians(0)));

		// Array of Tag specific Offsets, For quick refernce Array index is Tag ID
		// Values are robot centric, x + is away from the reef, y + is right
		// First Pair is the Left offset, second pair is the right.
		public static final double[][] tagSpecificOffset = { { 0, 0, 0, 0 }, // Tag ID 0
				{ 0, 0, 0, 0 }, // Tag ID 1
				{ 0, 0, 0, 0 }, // Tag ID 2
				{ 0, 0, 0, 0 }, // Tag ID 3
				{ 0, 0, 0, 0 }, // Tag ID 4
				{ 0, 0, 0, 0 }, // Tag ID 5
				{ 0, 0, 0, 0 }, // Tag ID 6
				{ 0, 0, 0, 0 }, // Tag ID 7
				{ 0, 0, 0, 0 }, // Tag ID 8
				{ 0, 0, 0, 0 }, // Tag ID 9
				{ 0, 0, 0, 0 }, // Tag ID 10
				{ 0, 0, 0, 0 }, // Tag ID 11
				{ 0, 0, 0, 0 }, // Tag ID 12
				{ 0, 0, 0, 0 }, // Tag ID 13
				{ 0, 0, 0, 0 }, // Tag ID 14
				{ 0, 0, 0, 0 }, // Tag ID 15
				{ 0, 0, 0, 0 }, // Tag ID 16
				{ 0, 0, 0, 0 }, // Tag ID 17
				{ 0, 0, 0, 0 }, // Tag ID 18
				{ 0, 0, 0, 0 }, // Tag ID 19
				{ 0, 0, 0, 0 }, // Tag ID 20
				{ 0, 0, 0, 0 }, // Tag ID 21
				{ 0, 0, 0, 0 } // Tag ID 22
		};

		public static final double AngTolerance = 25.0 / 180 * Math.PI;

		// Cam mounted facing forward, half a meter forward of center, half a meter up
		// from center.
		public static final Transform3d kRobotToCamFrontRight = new Transform3d(
				new Translation3d(Units.inchesToMeters(-4), Units.inchesToMeters(12.0), Units.inchesToMeters(10)),
				new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(27.5),
						Units.degreesToRadians(80.0)));

		public static final Transform3d kRobotToCamFrontLeft = new Transform3d(
				new Translation3d(Units.inchesToMeters(-11.099), Units.inchesToMeters(-13.19),
						Units.inchesToMeters(12.6)),
				new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(20), Units.degreesToRadians(-90)));

		public static final double kMaxGyroCameraAngleDelta = 89.0;
	}

	public class VisionTargetTag {
		public int tagID;
		public Pose2d pose;
	}

	private final AprilTagFieldLayout aprilTagFieldLayout;
	private final PhotonCamera frontRightCam;
	private final PhotonCamera frontLeftCam;
	private final DriveSubsystem m_driveSubsystem;
	private final TurretSubsystem turretSubsystem;
	private Field2d fieldDisplay = new Field2d();
	private List<Pose2d> blueTargets = new ArrayList<>();
	private List<Pose2d> redTargets = new ArrayList<>();
	private VisionTargetTag target = new VisionTargetTag();

	// Construct PhotonPoseEstimator
	public final PhotonPoseEstimator frontRightPhotonPoseEstimator;
	public final PhotonPoseEstimator frontLeftPhotonPoseEstimator;

	public VisionSubsystem(DriveSubsystem driveSubsystem, TurretSubsystem turretSubsystem, HardwareMonitor hardwareMonitor) {
		m_driveSubsystem = driveSubsystem;
		this.turretSubsystem = turretSubsystem;
		aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

		frontRightCam = new PhotonCamera(VisionConstants.kFrontRightCameraName);
		frontLeftCam = new PhotonCamera(VisionConstants.kFrontLeftCameraName);

		frontRightPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, VisionConstants.kRobotToCamFrontRight);

		frontLeftPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, VisionConstants.kRobotToCamFrontLeft);

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

		hardwareMonitor.registerDevice(this, frontLeftCam);
		hardwareMonitor.registerDevice(this, frontRightCam);
	}

	/**
	 * Update the pose estimator with the pending results from a single camera.
	 */
	private void updateCamera(PhotonCamera cam, PhotonPoseEstimator estimator, String poseName) {
		final List<PhotonPipelineResult> results = cam.getAllUnreadResults();

		if (results.size() > 0) {
			var lastResult = results.get(results.size() - 1);
			Optional<EstimatedRobotPose> estimatedPose = estimator.estimateClosestToReferencePose(lastResult, new Pose3d(m_driveSubsystem.getPose()));
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
	}

	public Pose2d getRedTowerPose() {
		return VisionSubsystem.VisionConstants.redTower;
	}

	public Pose2d getClosestReefTargetRed() {
		double RobotRearOrientation = m_driveSubsystem.getPose().getRotation().getRadians() - Math.PI;
		target.tagID = 0;

		for (int i = 0; i < redTargets.size(); i++) {
			if (Math.cos(redTargets.get(i).getRotation().getRadians() - RobotRearOrientation) > Math
					.cos(VisionConstants.AngTolerance)) {
				target.pose = redTargets.get(i);
				target.tagID = i + 6;
				return target.pose;
			}
		}
		target.pose = m_driveSubsystem.getPose().nearest(redTargets);
		for (int i = 0; i < redTargets.size(); i++) {
			if (target.pose.getX() == redTargets.get(i).getX() && target.pose.getY() == redTargets.get(i).getY())
				target.tagID = i + 6;
		}
		return target.pose;
	}

	public Pose2d getClosestReefTargetBlue() {
		double RobotRearOrientation = m_driveSubsystem.getPose().getRotation().getRadians() - Math.PI;
		target.tagID = 0;

		for (int i = 0; i < blueTargets.size(); i++) {
			if (Math.cos(blueTargets.get(i).getRotation().getRadians() - RobotRearOrientation) > Math
					.cos(VisionConstants.AngTolerance)) {
				target.pose = blueTargets.get(i);
				target.tagID = i + 17;
				return target.pose;
			}
		}
		target.pose = m_driveSubsystem.getPose().nearest(blueTargets);
		for (int i = 0; i < blueTargets.size(); i++) {
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
		updateCamera(frontRightCam, frontRightPhotonPoseEstimator, "RightFrontPose");

		updateCamera(frontLeftCam, frontLeftPhotonPoseEstimator, "LeftFrontPose");

		fieldDisplay.setRobotPose(m_driveSubsystem.getPose());
		fieldDisplay.getObject("blueHub")
				.setPose(TurretSubsystem.getHub().plus(TurretSubsystem.changeTargetLocation("67")));
		fieldDisplay.getObject("Turret").setPose(turretSubsystem.getTurretPose());
		fieldDisplay.getObject("Shoot On The Fly Pose").setPose(m_driveSubsystem.getShootOnTheFlyPose2d());

	}
}