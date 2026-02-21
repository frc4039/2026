// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class IntakeSlideSubsystem extends SubsystemBase {
	public final class IntakeSlideSubsystemConstants {

		//Motor Velocity used for running the motors(into the limit switch)
		public static final double KMotorVelocity = 20;

		// Device ids
		public static final int kLeaderMotorID = 22;
		public static final int kFollowerMotorID = 23;

		// Pid values
		public static final double kVIntakeSlide = 0.0;
		public static final double kSIntakeSlide = 0.0;
		public static final double kAIntakeSlide = 0.0;
		public static final double kPIntakeSlide = 1.0;
		public static final double kIIntakeSlide = 0.0;
		public static final double kDIntakeSlide = 0.0;

		//Constants for intake in and out positions
		public static final double kOutPosition = -19.0;
		public static final double kInPosition = 0.0;

		// Used for determining when the command ends.
		public static final double kPositionThreshold = 0.5;

		// Motion magic
		public static final double kCruiseVelocity = 80.0;
		public static final double kAcceleration = 200.0;
		public static final double kJerk = 800.0;

		//PWM Ports for intake limit switches
		public static final int kLeftLimitSwitchChannel = 0;
		public static final int kRightLimitSwitchChannel = 1;

	}

	//Motors and limit Switches
	public TalonFX intakeSlideLeftMotor, intakeSlideRightMotor;
	private final DigitalInput limitSwitchLeft, limitSwitchRight;


		// private double manualVelocity = 10.0;

		// private VoltageOut voltRequest = new VoltageOut(0.0);
		// private SysIdRoutine sysid = new SysIdRoutine(
		// 		new SysIdRoutine.Config(
		// 				Volts.of(0.5).per(Second),
		// 				Volts.of(4),
		// 				Seconds.of(5),
		// 				(state) -> SignalLogger.writeString("state", state.toString())),
		// 		new SysIdRoutine.Mechanism(
		// 				(volts) -> intakeSlideLeftMotor.setControl(voltRequest.withOutput(volts.in(Volts))),
		// 				null,
		// 				this));

	public IntakeSlideSubsystem() {
		//Sets the CanIDs for each respective motor
		intakeSlideLeftMotor = new TalonFX(IntakeSlideSubsystemConstants.kLeaderMotorID);
		intakeSlideRightMotor = new TalonFX(IntakeSlideSubsystemConstants.kFollowerMotorID);

		//Sets each limitswitch to its respective PWM port
		limitSwitchLeft = new DigitalInput(IntakeSlideSubsystemConstants.kLeftLimitSwitchChannel);
		limitSwitchRight = new DigitalInput(IntakeSlideSubsystemConstants.kRightLimitSwitchChannel);

		//Motor Output Configs
		MotorOutputConfigs mcfgLeaderMotor = new MotorOutputConfigs();
		MotorOutputConfigs mcfgFollowerMotor = new MotorOutputConfigs();

		mcfgLeaderMotor.withInverted(InvertedValue.CounterClockwise_Positive);
		mcfgLeaderMotor.withNeutralMode(NeutralModeValue.Coast);
		mcfgFollowerMotor.withInverted(InvertedValue.Clockwise_Positive);
		mcfgFollowerMotor.withNeutralMode(NeutralModeValue.Coast);

		TalonFXConfiguration talonFXConfigurationLeader = new TalonFXConfiguration().withMotorOutput(mcfgLeaderMotor);
		TalonFXConfiguration talonFXConfigurationFollower = new TalonFXConfiguration()
				.withMotorOutput(mcfgFollowerMotor);

		Slot0Configs slotConfigsLeader = talonFXConfigurationLeader.Slot0;
		Slot0Configs slotConfigsFollower = talonFXConfigurationFollower.Slot0;

		//PID Tuning
		slotConfigsLeader.kS = IntakeSlideSubsystemConstants.kSIntakeSlide;
		slotConfigsLeader.kV = IntakeSlideSubsystemConstants.kVIntakeSlide;
		slotConfigsLeader.kA = IntakeSlideSubsystemConstants.kAIntakeSlide;
		slotConfigsLeader.kP = IntakeSlideSubsystemConstants.kPIntakeSlide;
		slotConfigsLeader.kI = IntakeSlideSubsystemConstants.kIIntakeSlide;
		slotConfigsLeader.kD = IntakeSlideSubsystemConstants.kDIntakeSlide;

		slotConfigsFollower.kS = IntakeSlideSubsystemConstants.kSIntakeSlide;
		slotConfigsFollower.kV = IntakeSlideSubsystemConstants.kVIntakeSlide;
		slotConfigsFollower.kA = IntakeSlideSubsystemConstants.kAIntakeSlide;
		slotConfigsFollower.kP = IntakeSlideSubsystemConstants.kPIntakeSlide;
		slotConfigsFollower.kI = IntakeSlideSubsystemConstants.kIIntakeSlide;
		slotConfigsFollower.kD = IntakeSlideSubsystemConstants.kDIntakeSlide;

		MotionMagicConfigs motionMagicConfigsLeader = talonFXConfigurationLeader.MotionMagic;
		MotionMagicConfigs motionMagicConfigsFollower = talonFXConfigurationFollower.MotionMagic;

		motionMagicConfigsLeader.MotionMagicCruiseVelocity = IntakeSlideSubsystemConstants.kCruiseVelocity;
		motionMagicConfigsLeader.MotionMagicAcceleration = IntakeSlideSubsystemConstants.kAcceleration;
		motionMagicConfigsLeader.MotionMagicJerk = IntakeSlideSubsystemConstants.kJerk;

		motionMagicConfigsFollower.MotionMagicCruiseVelocity = IntakeSlideSubsystemConstants.kCruiseVelocity;
		motionMagicConfigsFollower.MotionMagicAcceleration = IntakeSlideSubsystemConstants.kAcceleration;
		motionMagicConfigsFollower.MotionMagicJerk = IntakeSlideSubsystemConstants.kJerk;

		intakeSlideLeftMotor.getConfigurator().apply(talonFXConfigurationLeader);
		intakeSlideRightMotor.getConfigurator().apply(talonFXConfigurationFollower);
	}

	//Runs both motors at a set velocity taking in direction
	public void run(Boolean forward) {
		final VelocityVoltage request = new VelocityVoltage(IntakeSlideSubsystemConstants.KMotorVelocity).withSlot(0);

		if (forward) {
			intakeSlideLeftMotor.setControl(request.withVelocity(IntakeSlideSubsystemConstants.KMotorVelocity));
		} else {
			intakeSlideLeftMotor.setControl(request.withVelocity(-1 * IntakeSlideSubsystemConstants.KMotorVelocity));
		}
	}

	//Takes a velocity and moves the motors at that velocity
	public void moveInput(double velocity) {
		final VelocityVoltage request = new VelocityVoltage(velocity).withSlot(0);
		intakeSlideLeftMotor.setControl(request.withVelocity(velocity));
	}

	//Stops the motors
	public void stop() {
		intakeSlideLeftMotor.stopMotor();
		intakeSlideRightMotor.stopMotor();
	}

	//Drives the motors inwards until they hit the limit
	public void driveUntilLimit() {
		if (limitSwitchLeft.get()) {
			intakeSlideLeftMotor.set(0.07);
		} else {
			intakeSlideLeftMotor.stopMotor();
			intakeSlideLeftMotor.setPosition(0);
		}

		if (limitSwitchRight.get()) {
			intakeSlideRightMotor.set(0.07);
		} else {
			intakeSlideRightMotor.stopMotor();
			intakeSlideRightMotor.setPosition(0);
		}
	}

	//Moves the motors to a desired position
	public void moveToPosition(double position) {
		final MotionMagicVoltage request = new MotionMagicVoltage(position);

		intakeSlideRightMotor.setControl(request.withPosition(position)
				.withSlot(0).withLimitForwardMotion(!limitSwitchRight.get())
				.withOverrideBrakeDurNeutral(true));

		intakeSlideLeftMotor.setControl(request.withPosition(position)
				.withSlot(0).withLimitForwardMotion(!limitSwitchLeft.get())
				.withOverrideBrakeDurNeutral(true));
	}

	//Sets the intakes position to zero
	public void zeroIntake() {
		intakeSlideLeftMotor.setPosition(0);
		intakeSlideRightMotor.setPosition(0);
	}

	//Used to reset the intake slide motors when they hit the limit
	@Override
	public void periodic() {
		// For safety, stop the turret whenever the robot is disabled.
		if (!limitSwitchLeft.get()) {
			intakeSlideLeftMotor.setPosition(0);
		}

		if (!limitSwitchRight.get()) {
			intakeSlideRightMotor.setPosition(0);
		}
	}

	//Sends data to the Dashboard
	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Intake Slide Left Position",
				() -> intakeSlideLeftMotor.getPosition().getValueAsDouble(), null);
		builder.addDoubleProperty("Intake Slide Right Position",
				() -> intakeSlideRightMotor.getPosition().getValueAsDouble(), null);
		builder.addBooleanProperty("Left Limit Switch", () -> limitSwitchLeft.get(), null);
		builder.addBooleanProperty("Right Limit Switch", () -> limitSwitchRight.get(), null);
		builder.addDoubleProperty("Right Motor Current",
				() -> intakeSlideRightMotor.getStatorCurrent().getValueAsDouble(), null);
		builder.addDoubleProperty("Left Motor Current",
				() -> intakeSlideLeftMotor.getStatorCurrent().getValueAsDouble(), null);
	}

}
