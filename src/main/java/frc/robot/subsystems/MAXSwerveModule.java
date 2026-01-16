// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.utils.HardwareMonitor;
import frc.robot.utils.Helpers;

public class MAXSwerveModule {
	public static final class ModuleConstants {
		// The MAXSwerve module can be configured with one of three pinion gears: 12T,
		// 13T, or 14T. This changes the drive speed of the module (a pinion gear with
		// more teeth will result in a robot that drives faster).
		public static final int kDrivingMotorPinionTeeth = 14;

		// Invert the turning encoder, since the output shaft rotates in the opposite
		// direction of the steering motor in the MAXSwerve Module.
		public static final boolean kTurningEncoderInverted = true;

		// Calculations required for driving motor conversion factors and feed forward
		public static final double kDrivingMotorFreeSpeedRPM = 6000;
		public static final double kDrivingMotorFreeSpeedRps = kDrivingMotorFreeSpeedRPM / 60;
		public static final double kWheelDiameterInches = 3.00;
		public static final double kWheelDiameterMeters = kWheelDiameterInches * 0.0254;
		public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
		// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
		// teeth on the bevel pinion.
		public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
		public static Double kDrivingRotationsToMeters = (1.0 / kDrivingMotorReduction) * kWheelCircumferenceMeters;
		public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
				/ kDrivingMotorReduction;
		public static final double kDriveWheelFreeSpeedFps = kDriveWheelFreeSpeedRps * 3.28084; // conversion from m/s
																								// to ft/s because i got
																								// tired of doing the
																								// conversion every time
																								// ben asked
		// might be able to remove drive speed math - will check at later date
		public static final double kMpsToPercentOutput = kDrivingMotorReduction * 60 / kWheelCircumferenceMeters
				/ kDrivingMotorFreeSpeedRPM; // Multiply by meters per second to get percent output

		public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
				/ kDrivingMotorReduction; // meters
		public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
				/ kDrivingMotorReduction) / 60.0; // meters per second

		public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
		public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 30.0; // radians per second used to
																							// be /60

		public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
		public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

		public static final double kDrivingP = Helpers.isBabycakes() ? 0.0095 : 0.00965; // DO NOT CHANGE
		public static final double kDrivingI = 0;
		public static final double kDrivingD = 0;
		public static final double kDrivingFF = Helpers.isBabycakes() ? 0.016 : 0.0187; // ONLY INCREASE BY INCRMENTS OF
																						// 0.002 (0.22 for 100lb)
		public static final double kDrivingMinOutput = -1;
		public static final double kDrivingMaxOutput = 1;

		public static final double kTurningP = 1.1;
		public static final double kTurningI = 0;
		public static final double kTurningD = 0;
		public static final double kTurningFF = 0;// DO NOT INCREASE
		public static final double kTurningMinOutput = -1;
		public static final double kTurningMaxOutput = 1;

		public static final NeutralModeValue kDrivingMotorNeutralMode = NeutralModeValue.Brake;
		public static final double kDrivingMotorNeutralDeadband = 0.005; // Jagon Baker did this

		public static final int kDrivingMotorCurrentLimitHigh = 55; // Amps Before Limit is Applied
		public static final double kTriggerThresholdTime = 0.4; // Seconds Before Low Limit is Applied
		public static final int kDrivingMotorCurrentLimitLow = 40; // Amps After Threshold is Passed
		public static final int kTurningMotorCurrentLimit = 20; // Amps

		public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
		public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

		static {
			// Use module constants to calculate conversion factors and feed forward gain.
			double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
					/ ModuleConstants.kDrivingMotorReduction;
			double turningFactor = 2 * Math.PI;
			double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

			drivingConfig
					.idleMode(IdleMode.kBrake)
					.smartCurrentLimit(50);
			drivingConfig.encoder
					.positionConversionFactor(drivingFactor) // meters
					.velocityConversionFactor(drivingFactor / 60.0); // meters per second
			drivingConfig.closedLoop
					.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
					// These are example gains you may need to them for your own robot!
					.pid(0.04, 0, 0)
					.velocityFF(drivingVelocityFeedForward)
					.outputRange(-1, 1);

			turningConfig
					.idleMode(IdleMode.kBrake)
					.smartCurrentLimit(20);
			turningConfig.absoluteEncoder
					// Invert the turning encoder, since the output shaft rotates in the opposite
					// direction of the steering motor in the MAXSwerve Module.
					.inverted(true)
					.positionConversionFactor(turningFactor) // radians
					.velocityConversionFactor(turningFactor / 60.0); // radians per second
			turningConfig.closedLoop
					.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
					// These are example gains you may need to them for your own robot!
					.pid(1, 0, 0)
					.outputRange(-1, 1)
					// Enable PID wrap around for the turning motor. This will allow the PID
					// controller to go through 0 to get to the setpoint i.e. going from 350 degrees
					// to 10 degrees will go through 0 rather than the other direction which is a
					// longer route.
					.positionWrappingEnabled(true)
					.positionWrappingInputRange(0, turningFactor);
		}
	}

	private final TalonFX m_drivingTalonFx;
	// private final CANSparkMax m_turningSparkMax;
	private final SparkMax m_turningSparkMax;

	private final AbsoluteEncoder m_turningEncoder;

	private final SparkClosedLoopController m_turningClosedLoopController;

	private double m_chassisAngularOffset = 0;
	private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

	/**
	 * Constructs a MAXSwerveModule and configures the driving and turning motor,
	 * encoder, and PID controller. This configuration is specific to the REV
	 * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
	 * Encoder.
	 */
	public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
		m_drivingTalonFx = new TalonFX(drivingCANId);
		m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);

		TalonFXConfiguration m_drivingConfig = new TalonFXConfiguration();

		// Setup PID Controllers
		m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();
		m_turningClosedLoopController = m_turningSparkMax.getClosedLoopController();

		m_turningSparkMax.configure(ModuleConstants.turningConfig, ResetMode.kResetSafeParameters,
				PersistMode.kPersistParameters);

		m_drivingConfig.Slot0.kP = ModuleConstants.kDrivingP;
		m_drivingConfig.Slot0.kI = ModuleConstants.kDrivingI;
		m_drivingConfig.Slot0.kD = ModuleConstants.kDrivingD;
		m_drivingConfig.Slot0.kV = ModuleConstants.kDrivingFF;
		m_drivingConfig.MotorOutput.PeakReverseDutyCycle = ModuleConstants.kDrivingMinOutput;
		m_drivingConfig.MotorOutput.PeakForwardDutyCycle = ModuleConstants.kDrivingMaxOutput;

		m_drivingConfig.MotorOutput.NeutralMode = ModuleConstants.kDrivingMotorNeutralMode;
		m_drivingConfig.MotorOutput.DutyCycleNeutralDeadband = ModuleConstants.kDrivingMotorNeutralDeadband;
		m_drivingConfig.CurrentLimits.SupplyCurrentLimit = ModuleConstants.kDrivingMotorCurrentLimitHigh;
		m_drivingConfig.CurrentLimits.SupplyCurrentLowerLimit = ModuleConstants.kDrivingMotorCurrentLimitLow;
		m_drivingConfig.CurrentLimits.SupplyCurrentLowerTime = ModuleConstants.kTriggerThresholdTime;

		m_drivingTalonFx.getConfigurator().apply(m_drivingConfig);

		m_chassisAngularOffset = chassisAngularOffset;
		m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
		m_drivingTalonFx.setPosition(0);
	}

	/**
	 * Returns the current state of the module.
	 *
	 * @return The current state of the module.
	 */
	public SwerveModuleState getState() {
		// Apply chassis angular offset to the encoder position to get the position
		// relative to the chassis.
		return new SwerveModuleState(
				m_drivingTalonFx.getVelocity().getValueAsDouble() * ModuleConstants.kDrivingEncoderPositionFactor,
				new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
	}

	/**
	 * Returns the current position of the module.
	 *
	 * @return The current position of the module.
	 */
	public SwerveModulePosition getPosition() {
		// Apply chassis angular offset to the encoder position to get the position
		// relative to the chassis.
		return new SwerveModulePosition(
				m_drivingTalonFx.getPosition().getValueAsDouble() * ModuleConstants.kDrivingEncoderPositionFactor,
				new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
	}

	public double getWheelRadians() {
		return m_drivingTalonFx.getPosition().getValueAsDouble()
				* (2 * Math.PI / ModuleConstants.kDrivingMotorReduction);
	}

	/**
	 * Sets the desired state for the module.
	 *
	 * @param desiredState Desired state with speed and angle.
	 */
	public void setDesiredState(SwerveModuleState desiredState) {
		// Apply chassis angular offset to the desired state.
		SwerveModuleState correctedDesiredState = new SwerveModuleState();
		correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
		correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

		// Optimize the reference state to avoid spinning further than 90 degrees.
		correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

		// Command driving and turning SPARKS MAX towards their respective setpoints.
		m_drivingTalonFx.setControl(new VelocityDutyCycle(
				correctedDesiredState.speedMetersPerSecond / ModuleConstants.kDrivingEncoderPositionFactor));
		m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

		m_desiredState = desiredState;
	}

	/** Zeroes all the SwerveModule encoders. */
	public void resetEncoders() {
		m_drivingTalonFx.setPosition(0);
	}

	/** Register this module so its motors will be monitored. */
	public void registerWithHardwareTracker(Subsystem parent, HardwareMonitor hw) {
		hw.registerDevice(parent, m_drivingTalonFx);
		hw.registerDevice(parent, m_turningSparkMax);
	}
}
