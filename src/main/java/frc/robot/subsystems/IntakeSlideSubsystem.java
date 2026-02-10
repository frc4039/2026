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
		// How fast the turret rotates from one point to another.
		public static final double KMotorVelocity = 20;

		// Device ids
		public static final int kLeaderMotorID = 17;
		public static final int kFollowerMotorID = 18;

		// Pid values
		public static final double kVIntakeSlide = 0.0;
		public static final double kSIntakeSlide = 0.0;
		public static final double kAIntakeSlide = 0.0;
		public static final double kPIntakeSlide = 0.0;
		public static final double kIIntakeSlide = 0.0;
		public static final double kDIntakeSlide = 0.0;

    public static final double kDegreesPerRotation = 0.0;

    public static final double kOutPosition = 0.0;
    public static final double kInPosition = 0.0;

    // Motion magic
		public static final double kCruiseVelocity = 0.0 / kDegreesPerRotation;
		public static final double kAcceleration = 0.0 / kDegreesPerRotation;
		public static final double kJerk = 0.0 / kDegreesPerRotation;

		public static final int kLeftLimitSwitchChannel = 0;
		public static final int kRightLimitSwitchChannel = 1;

	}

	private TalonFX intakeSlideLeftMotor, intakeSlideRightMotor;
	private final DigitalInput limitSwitchLeft, limitSwitchRight;
	private double manualVelocity = 10.0;

	private VoltageOut voltRequest = new VoltageOut(0.0);
	private SysIdRoutine sysid = new SysIdRoutine(
			new SysIdRoutine.Config(
				Volts.of(0.5).per(Second),
				Volts.of(4),
				Seconds.of(5),
				(state) -> SignalLogger.writeString("state", state.toString())
			),
			new SysIdRoutine.Mechanism(
					(volts) -> intakeSlideLeftMotor.setControl(voltRequest.withOutput(volts.in(Volts))),
					null,
					this));

	public IntakeSlideSubsystem() {
		intakeSlideLeftMotor = new TalonFX(IntakeSlideSubsystemConstants.kLeaderMotorID);
		intakeSlideRightMotor = new TalonFX(IntakeSlideSubsystemConstants.kFollowerMotorID);

		limitSwitchLeft = new DigitalInput(IntakeSlideSubsystemConstants.kLeftLimitSwitchChannel);
		limitSwitchRight = new DigitalInput(IntakeSlideSubsystemConstants.kRightLimitSwitchChannel);

		
		//var follower = new Follower(IntakeSlideSubsystemConstants.kLeaderMotorID, MotorAlignmentValue.Opposed);

		//intakeSlideLeftMotor.setControl(follower);
		MotorOutputConfigs mcfgLeaderMotor = new MotorOutputConfigs();
		MotorOutputConfigs mcfgFollowerMotor = new MotorOutputConfigs();


		mcfgLeaderMotor.withInverted(InvertedValue.CounterClockwise_Positive);
		mcfgLeaderMotor.withNeutralMode(NeutralModeValue.Coast);
		mcfgFollowerMotor.withInverted(InvertedValue.Clockwise_Positive);
		mcfgFollowerMotor.withNeutralMode(NeutralModeValue.Coast);

		TalonFXConfiguration talonFXConfigurationLeader = new TalonFXConfiguration().withMotorOutput(mcfgLeaderMotor);
		TalonFXConfiguration talonFXConfigurationFollower = new TalonFXConfiguration().withMotorOutput(mcfgFollowerMotor);


		Slot0Configs slotConfigsLeader = talonFXConfigurationLeader.Slot0;
		Slot0Configs slotConfigsFollower = talonFXConfigurationFollower.Slot0;


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

	public void run(Boolean forward) {
		final VelocityVoltage request = new VelocityVoltage(IntakeSlideSubsystemConstants.KMotorVelocity).withSlot(0);

		if (forward) {
			intakeSlideLeftMotor.setControl(request.withVelocity(IntakeSlideSubsystemConstants.KMotorVelocity));
		} else {
			intakeSlideLeftMotor.setControl(request.withVelocity(-1 * IntakeSlideSubsystemConstants.KMotorVelocity));
		}
	}

	public void moveInput(double velocity) {
		final VelocityVoltage request = new VelocityVoltage(velocity).withSlot(0);
		intakeSlideLeftMotor.setControl(request.withVelocity(velocity));
	}

	public void stop() {
		intakeSlideLeftMotor.stopMotor();
	}

	public void driveUntilLimit() {
		if (limitSwitchLeft.get()) {
			intakeSlideLeftMotor.set(-0.5);
		} else {
			intakeSlideLeftMotor.stopMotor();
			intakeSlideLeftMotor.setPosition(0);
		}
		 
		if(limitSwitchRight.get()) {
			intakeSlideRightMotor.set(-0.5);
		} else {
			intakeSlideRightMotor.stopMotor();
			intakeSlideRightMotor.setPosition(0);
		}
	}

  public void moveToPosition(double position) {
    final MotionMagicVoltage request = new MotionMagicVoltage(position);

		final double newPosition = position / IntakeSlideSubsystemConstants.kDegreesPerRotation;

		intakeSlideLeftMotor.setControl(request.withPosition(newPosition)
				.withSlot(0)
				.withOverrideBrakeDurNeutral(true));  
		
		intakeSlideRightMotor.setControl(request.withPosition(newPosition)
				.withSlot(0)
				.withOverrideBrakeDurNeutral(true)); 
	}

	@Override
	public void periodic() {
		// For safety, stop the turret whenever the robot is disabled.
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Intake Slide Left Position", () -> intakeSlideLeftMotor.getPosition().getValueAsDouble(), null);
		builder.addDoubleProperty("Intake Slide Right Position", () -> intakeSlideRightMotor.getPosition().getValueAsDouble(), null);
		builder.addBooleanProperty("Left Limit Switch", () -> limitSwitchLeft.get(), null);
		builder.addBooleanProperty("Right Limit Switch", () -> limitSwitchRight.get(), null);
		builder.addDoubleProperty("Right Motor Current", () -> intakeSlideRightMotor.getStatorCurrent().getValueAsDouble(), null);
		builder.addDoubleProperty("Left Motor Current", () -> intakeSlideLeftMotor.getStatorCurrent().getValueAsDouble(), null);
  }

}

