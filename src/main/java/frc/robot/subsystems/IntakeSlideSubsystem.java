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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.TurretSubsystem.TurretConstants;

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

	}

	private TalonFX intakeSlideLeaderMotor, intakeSlideFollowerMotor;

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
					(volts) -> intakeSlideLeaderMotor.setControl(voltRequest.withOutput(volts.in(Volts))),
					null,
					this));

	public IntakeSlideSubsystem() {
		intakeSlideLeaderMotor = new TalonFX(IntakeSlideSubsystemConstants.kLeaderMotorID);
		intakeSlideFollowerMotor = new TalonFX(IntakeSlideSubsystemConstants.kFollowerMotorID);

		intakeSlideLeaderMotor.setNeutralMode(NeutralModeValue.Brake);
		intakeSlideFollowerMotor.setNeutralMode(NeutralModeValue.Brake);

		var follower = new Follower(IntakeSlideSubsystemConstants.kLeaderMotorID, MotorAlignmentValue.Opposed);

		intakeSlideLeaderMotor.setControl(follower);
		MotorOutputConfigs mcfg = new MotorOutputConfigs();

		mcfg.withInverted(InvertedValue.CounterClockwise_Positive);
		mcfg.withNeutralMode(NeutralModeValue.Coast);

		TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration().withMotorOutput(mcfg);

		Slot0Configs slotConfigs = talonFXConfigs.Slot0;

		slotConfigs.kS = IntakeSlideSubsystemConstants.kSIntakeSlide;
		slotConfigs.kV = IntakeSlideSubsystemConstants.kVIntakeSlide;
		slotConfigs.kA = IntakeSlideSubsystemConstants.kAIntakeSlide;
		slotConfigs.kP = IntakeSlideSubsystemConstants.kPIntakeSlide;
		slotConfigs.kI = IntakeSlideSubsystemConstants.kIIntakeSlide;
		slotConfigs.kD = IntakeSlideSubsystemConstants.kDIntakeSlide;

    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;

		motionMagicConfigs.MotionMagicCruiseVelocity = IntakeSlideSubsystemConstants.kCruiseVelocity;
		motionMagicConfigs.MotionMagicAcceleration = IntakeSlideSubsystemConstants.kAcceleration;
		motionMagicConfigs.MotionMagicJerk = IntakeSlideSubsystemConstants.kJerk;

		intakeSlideLeaderMotor.getConfigurator().apply(talonFXConfigs);
	}

	public void run(Boolean forward) {
		final VelocityVoltage request = new VelocityVoltage(IntakeSlideSubsystemConstants.KMotorVelocity).withSlot(0);

		if (forward) {
			intakeSlideLeaderMotor.setControl(request.withVelocity(IntakeSlideSubsystemConstants.KMotorVelocity));
		} else {
			intakeSlideLeaderMotor.setControl(request.withVelocity(-1 * IntakeSlideSubsystemConstants.KMotorVelocity));
		}
	}

	public void moveInput(double velocity) {
		final VelocityVoltage request = new VelocityVoltage(velocity).withSlot(0);
		intakeSlideLeaderMotor.setControl(request.withVelocity(velocity));
	}

	public void stop() {
		intakeSlideLeaderMotor.stopMotor();
	}

  public void moveToPosition(double position) {
    final MotionMagicVoltage request = new MotionMagicVoltage(position);

		final double newPosition = position / IntakeSlideSubsystemConstants.kDegreesPerRotation;

		intakeSlideLeaderMotor.setControl(request.withPosition(newPosition)
				.withSlot(0)
				.withOverrideBrakeDurNeutral(true));  }

	@Override
	public void periodic() {
		// For safety, stop the turret whenever the robot is disabled.
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Intake Slide Position", () -> intakeSlideLeaderMotor.getPosition().getValueAsDouble(), null);
  }

}

