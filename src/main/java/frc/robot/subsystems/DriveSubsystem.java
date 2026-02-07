// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.subsystems.TurretSubsystem.TurretConstants;
import frc.robot.utils.HardwareMonitor;
import frc.robot.utils.Helpers;
import frc.robot.utils.ShotCalculator;

public class DriveSubsystem extends SubsystemBase {
	public static final class DriveConstants {
		// Driving Parameters - Note that these are not the maximum capable speeds of
		// the robot, rather the allowed maximum speeds
		public static final double kMaxSpeedMetersPerSecond = 2.0; // was 5.0 //Keep Lower Than Real Max
		public static final double kMaxAngularSpeed = Helpers.isBabycakes() ? (1.35 * Math.PI) : (1.8 * Math.PI); // radians per second for turning

		public static final double kAimP = 1.8;
		public static final double kAimI = 0;
		public static final double kAimD = 0;
		public static final Constraints kAimProfile = new Constraints(6 * Math.PI, 4 * Math.PI);

		public static final double KTransP = 1.0;
		public static final double KTransI = 0.0;
		public static final double KTransD = 0.0;
		public static final Constraints KTransProfile = new Constraints(1, 1);
		public static final Constraints KRotProfile = new Constraints(Math.PI, Math.PI);

		// Chassis configuration
		public static final double kTrackWidth = Helpers.isBabycakes() ? Units.inchesToMeters(22.5)
				: Units.inchesToMeters(18.5);
		// Distance between centers of right and left wheels on robot
		public static final double kWheelBase = Helpers.isBabycakes() ? Units.inchesToMeters(22.5)
				: Units.inchesToMeters(18.5);
		public static final double kDriveBaseRadius = Helpers.isBabycakes() ? Units.inchesToMeters(15.9099)
				: Units.inchesToMeters(13.0815);

		// Distance between front and back wheels on robot
		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

		// Angular offsets of the modules relative to the chassis in radians
		public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
		public static final double kFrontRightChassisAngularOffset = 0;
		public static final double kBackLeftChassisAngularOffset = Math.PI;
		public static final double kBackRightChassisAngularOffset = Math.PI / 2;

		// Pigeon CAN ID
		public static final int kGyroID = 30;

		// KRAKEN X60 CAN IDs
		public static final int kFrontLeftDrivingCanId = 11;
		public static final int kRearLeftDrivingCanId = 13;
		public static final int kFrontRightDrivingCanId = 15;
		public static final int kRearRightDrivingCanId = 17;

		// SPARK MAX CAN IDs
		public static final int kFrontLeftTurningCanId = 10;
		public static final int kRearLeftTurningCanId = 12;
		public static final int kFrontRightTurningCanId = 14;
		public static final int kRearRightTurningCanId = 16;

		public static final boolean kGyroReversed = false;
	}

	// Create MAXSwerveModules
	private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
			DriveConstants.kFrontLeftDrivingCanId,
			DriveConstants.kFrontLeftTurningCanId,
			DriveConstants.kFrontLeftChassisAngularOffset);

	private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
			DriveConstants.kFrontRightDrivingCanId,
			DriveConstants.kFrontRightTurningCanId,
			DriveConstants.kFrontRightChassisAngularOffset);

	private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
			DriveConstants.kRearLeftDrivingCanId,
			DriveConstants.kRearLeftTurningCanId,
			DriveConstants.kBackLeftChassisAngularOffset);

	private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
			DriveConstants.kRearRightDrivingCanId,
			DriveConstants.kRearRightTurningCanId,
			DriveConstants.kBackRightChassisAngularOffset);

	// The gyro sensor
	private final Pigeon2 m_gyro = new Pigeon2(DriveConstants.kGyroID);

	// Slew rate filter variables for controlling lateral acceleration
	private double m_currentRotation = 0.0;
	
	private final ShotCalculator shotCalculator;

	// Odometry class for tracking robot pose
	public SwerveDrivePoseEstimator m_poseEstimator;

	/** Creates a new DriveSubsystem. */
	public DriveSubsystem(HardwareMonitor hw, ShotCalculator shotCalculator) {
		this.shotCalculator = shotCalculator;

		m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
				Rotation2d.fromDegrees(0.0),
				new SwerveModulePosition[] {
						m_frontLeft.getPosition(),
						m_frontRight.getPosition(),
						m_rearLeft.getPosition(),
						m_rearRight.getPosition()
				}, new Pose2d(),
				VecBuilder.fill(0.04, 0.04, 0.04),
				VecBuilder.fill(0.5, 0.5, 0.5));

		m_frontLeft.registerWithHardwareTracker(this, hw);
		m_frontRight.registerWithHardwareTracker(this, hw);
		m_rearLeft.registerWithHardwareTracker(this, hw);
		m_rearRight.registerWithHardwareTracker(this, hw);
		hw.registerDevice(this, m_gyro);

		// Configure AutoBuilder last
		RobotConfig config;

		try {
			config = RobotConfig.fromGUISettings();

			AutoBuilder.configure(
					this::getPose, // Robot pose supplier
					this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
					this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
					(speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given
																			// ROBOT RELATIVE ChassisSpeeds
					new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your
													// Constants class
							new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
							new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
					// 4.5, // Max module speed, in m/s
					// 0.4, // Drive base radius in meters. Distance from robot center to furthest
					// module.
					// new ReplanningConfig() // Default path replanning config. See the API for the
					// options here
					),
					config,
					() -> {
						// Boolean supplier that controls when the path will be mirrored for the red
						// alliance. This will flip the path being followed to the red side of the
						// field.
						// THE ORIGIN WILL REMAIN ON THE BLUE SIDE

						var alliance = DriverStation.getAlliance();
						if (alliance.isPresent()) {
							return alliance.get() == DriverStation.Alliance.Red;
						}
						return false;
					},
					this // Reference to this subsystem to set requirements
			);

		} catch (Exception e) {
			// Handle exception as needed
			e.printStackTrace();
		}

		var gyroConfig = new MountPoseConfigs();
		gyroConfig.MountPoseYaw = 0;
		gyroConfig.MountPosePitch = 0;
		gyroConfig.MountPoseRoll = 0;
		m_gyro.getConfigurator().apply(gyroConfig);
		m_gyro.setYaw(0);
		m_gyro.getAccelerationX().setUpdateFrequency(100);
		m_gyro.getAccelerationY().setUpdateFrequency(100);
		m_gyro.getAccelerationX().setUpdateFrequency(100);

		SmartDashboard.putData("Swerve Drive", new Sendable() {
			@Override
			public void initSendable(SendableBuilder builder) {
				builder.setSmartDashboardType("SwerveDrive");

				builder.addDoubleProperty("Front Left Angle", () -> m_frontLeft.getState().angle.getRadians(), null);
				builder.addDoubleProperty("Front Left Velocity", () -> m_frontLeft.getState().speedMetersPerSecond,
						null);

				builder.addDoubleProperty("Front Right Angle", () -> m_frontRight.getState().angle.getRadians(), null);
				builder.addDoubleProperty("Front Right Velocity", () -> m_frontRight.getState().speedMetersPerSecond,
						null);

				builder.addDoubleProperty("Back Left Angle", () -> m_rearLeft.getState().angle.getRadians(), null);
				builder.addDoubleProperty("Back Left Velocity", () -> m_rearLeft.getState().speedMetersPerSecond, null);

				builder.addDoubleProperty("Back Right Angle", () -> m_rearLeft.getState().angle.getRadians(), null);
				builder.addDoubleProperty("Back Right Velocity", () -> m_rearLeft.getState().speedMetersPerSecond,
						null);

				builder.addDoubleProperty("Robot Angle", () -> getPose().getRotation().getRadians(), null);
			}
		});
	}

	@Override
	public void periodic() {
		// Update the odometry in the periodic block
		m_poseEstimator.update(
				Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
				new SwerveModulePosition[] {
						m_frontLeft.getPosition(),
						m_frontRight.getPosition(),
						m_rearLeft.getPosition(),
						m_rearRight.getPosition()
				});

		shotCalculator.update(new Pose3d(getPose()), getVelocity());
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Angle", () -> getPose().getRotation().getDegrees(), null);
		builder.addDoubleProperty("X Meters", () -> getPose().getX(), null);
		builder.addDoubleProperty("Y Meters", () -> getPose().getY(), null);
		builder.addDoubleProperty("Speed (mps)", () -> getSpeed(), null);
		builder.addDoubleProperty("Speed (fps)", () -> Units.metersToFeet(getSpeed()), null);
		builder.addDoubleProperty("Acceleration", () -> getAcceleration(), null);
		builder.addDoubleProperty("Robot Velocity X", () -> this.getRobotFieldSpeedX(), null);
		builder.addDoubleProperty("Robot Velocity Y", () -> this.getRobotFieldSpeedY(), null);
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return m_poseEstimator.getEstimatedPosition();
	}

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {
		m_poseEstimator.resetPosition(
				Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
				new SwerveModulePosition[] {
						m_frontLeft.getPosition(),
						m_frontRight.getPosition(),
						m_rearLeft.getPosition(),
						m_rearRight.getPosition()
				},
				pose);
	}

	/**
	 * Reset the gyro to the specified anglek, in degrees.
	 * 
	 * @param degrees The angle to set the gyro to.
	 */
	public void resetGyroAngle(Pose2d pose) {
		m_gyro.setYaw(pose.getRotation().getDegrees());
		m_poseEstimator.resetPosition(
				pose.getRotation(),
				new SwerveModulePosition[] {
						m_frontLeft.getPosition(),
						m_frontRight.getPosition(),
						m_rearLeft.getPosition(),
						m_rearRight.getPosition()
				},
				pose);
	}

	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed        Speed of the robot in the x direction (forward).
	 * @param ySpeed        Speed of the robot in the y direction (sideways).
	 * @param rot           Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the
	 *                      field.
	 */
	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

		double xSpeedCommanded;
		double ySpeedCommanded;

		xSpeedCommanded = xSpeed;
		ySpeedCommanded = ySpeed;
		m_currentRotation = rot;

		// Convert the commanded speeds into the correct units for the drivetrain
		double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
		double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
		double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

		if (fieldRelative == true) {
			// Speed needs to always be away from the alliance wall.
			Optional<Alliance> alliance = DriverStation.getAlliance();
			if (alliance.isPresent() && alliance.get() == Alliance.Red) {
				xSpeedDelivered = -xSpeedDelivered;
				ySpeedDelivered = -ySpeedDelivered;
			}
		}

		driveRaw(xSpeedDelivered, ySpeedDelivered, rotDelivered, fieldRelative);
	}

	/**
	 * Drive with real units for the movement.
	 */
	public void driveRaw(double xSpeedMetersPerSecond, double ySpeedMetersPerSecond, double rotRadiansPerSecond,
			boolean fieldRelative) {
		var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
				fieldRelative
						? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond,
								rotRadiansPerSecond, getPose().getRotation())
						: new ChassisSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, rotRadiansPerSecond));
		SwerveDriveKinematics.desaturateWheelSpeeds(
				swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
		m_frontLeft.setDesiredState(swerveModuleStates[0]);
		m_frontRight.setDesiredState(swerveModuleStates[1]);
		m_rearLeft.setDesiredState(swerveModuleStates[2]);
		m_rearRight.setDesiredState(swerveModuleStates[3]);
	}

	/**
	 * Sets the wheels into an X formation to prevent movement.
	 */
	public void setX() {
		m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
		m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
	}

	public void setWheelRadiusCailbration() {
		m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
		m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
		m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
	}

	public double[] getSwerveModulePositions() {
		return new double[] {
				m_frontLeft.getWheelRadians(),
				m_frontRight.getWheelRadians(),
				m_rearLeft.getWheelRadians(),
				m_rearRight.getWheelRadians()
		};
	}

	/**
	 * Sets the swerve ModuleStates.
	 *
	 * @param desiredStates The desired SwerveModule states.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(
				desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
		m_frontLeft.setDesiredState(desiredStates[0]);
		m_frontRight.setDesiredState(desiredStates[1]);
		m_rearLeft.setDesiredState(desiredStates[2]);
		m_rearRight.setDesiredState(desiredStates[3]);
	}

	/** Resets the drive encoders to currently read a position of 0. */
	public void resetEncoders() {
		m_frontLeft.resetEncoders();
		m_rearLeft.resetEncoders();
		m_frontRight.resetEncoders();
		m_rearRight.resetEncoders();
	}

	/**
	 * Returns the heading of the robot.
	 *
	 * @return the robot's heading in degrees, from -180 to 180
	 */
	public double getHeading() {
		return getPose().getRotation().getDegrees();
	}

	/**
	 * Returns the turn rate of the robot.
	 *
	 * @return The turn rate of the robot, in degrees per second
	 */
	public double getTurnRate() {
		return m_gyro.getAngularVelocityZDevice().getValueAsDouble();
	}

	public void driveRobotRelative(ChassisSpeeds speeds) {
		this.driveRaw(
				speeds.vxMetersPerSecond,
				speeds.vyMetersPerSecond,
				speeds.omegaRadiansPerSecond,
				false);
	}

	public ChassisSpeeds getRobotRelativeSpeeds() {
		return DriveConstants.kDriveKinematics.toChassisSpeeds(
				new SwerveModuleState[] {
						m_frontLeft.getState(),
						m_frontRight.getState(),
						m_rearLeft.getState(),
						m_rearRight.getState()
				});
	}

	public Translation2d getTranslationToCorner() {
		var cornerposition = new Translation2d(0, 8);
		Optional<Alliance> alliance = DriverStation.getAlliance();
		if (alliance.isPresent() && alliance.get() == Alliance.Red) {
			cornerposition = new Translation2d(16.46, 8);
		}
		return getPose().getTranslation().minus(cornerposition);
	}

	public double getRobotFieldSpeedX() {
		double chassisSpeedsVectorAngle = Math.atan2(getRobotRelativeSpeeds().vyMetersPerSecond,
				getRobotRelativeSpeeds().vxMetersPerSecond);
		return Math.cos(getPose().getRotation().getRadians() + chassisSpeedsVectorAngle)
				* Math.sqrt((Math.pow(getRobotRelativeSpeeds().vxMetersPerSecond, 2.0)
						+ Math.pow(getRobotRelativeSpeeds().vyMetersPerSecond, 2.0)));
	}

	public double getRobotFieldSpeedY() {
		double chassisSpeedsVectorAngle = Math.atan2(getRobotRelativeSpeeds().vyMetersPerSecond,
				getRobotRelativeSpeeds().vxMetersPerSecond);
		return Math.sin(getPose().getRotation().getRadians() + chassisSpeedsVectorAngle)
				* Math.sqrt((Math.pow(getRobotRelativeSpeeds().vxMetersPerSecond, 2.0)
						+ Math.pow(getRobotRelativeSpeeds().vyMetersPerSecond, 2.0)));
	}

	public double getRobotVectorAngle() {
		return Math.atan2(getRobotFieldSpeedY(),
				getRobotFieldSpeedX());

	}

	/** Get the angle from the robot to the note */
	public double getAcceleration() {
		double robotAcceleration = Math.sqrt((Math.pow(m_gyro.getAccelerationX().getValueAsDouble(), 2.0)
				+ Math.pow(m_gyro.getAccelerationY().getValueAsDouble(), 2.0)
				+ Math.pow(m_gyro.getAccelerationZ().getValueAsDouble() - 1, 2.0)));
		return robotAcceleration;
	}

	public double getSpeed() {
		double robotSpeed = Math.sqrt((Math.pow(getRobotRelativeSpeeds().vxMetersPerSecond, 2.0))
				+ Math.pow(getRobotRelativeSpeeds().vyMetersPerSecond, 2.0));
		return robotSpeed;
	}

	public ChassisSpeeds getVelocity() {
		Pose2d currentPose = getPose();
		ChassisSpeeds robotRelativeSpeeds = getRobotRelativeSpeeds();
		ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, currentPose.getRotation());
		return fieldRelativeSpeeds;
	}

	/** Return x Pose Value */
	public double getPoseXValue() {
		return this.getPose().getX();
	}

	public double getPoseYValue() {
		return this.getPose().getY();
	}

	public void addVisionPose(Pose2d estVisionPose, double estVisionTimeStamp, String source) {
		m_poseEstimator.addVisionMeasurement(estVisionPose, estVisionTimeStamp);
	}

	public Pose2d getShootOnTheFlyPose2d() {
		Pose2d currentPose2d = this.getPose();
		return currentPose2d.plus(new Transform2d((this.getRobotRelativeSpeeds().vxMetersPerSecond * TurretConstants.kTimeOfFlight), (this.getRobotRelativeSpeeds().vyMetersPerSecond * TurretConstants.kTimeOfFlight), new Rotation2d(0)));
	}
	
	
	public void moveSimulation(double x, double y, double rotation) {
		Pose2d currentPose = getPose();
		
		Pose2d simPose = new Pose2d(currentPose.getX() + x * 0.2, currentPose.getY() + y * -0.2, currentPose.getRotation().plus(Rotation2d.fromDegrees(rotation * 2)));
		m_poseEstimator.resetPose(simPose);
	}

}
