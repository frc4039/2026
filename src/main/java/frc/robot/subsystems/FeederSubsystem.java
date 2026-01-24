package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.utils.Helpers;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class FeederSubsystem extends SubsystemBase {
	// Constants of the feeder subsystem
	public static final class FeederConstants {
		public static final int kFeederMotorId = 41;
		public static final double kFeederSpeed = -2200;

		public static final double kSpindexerWheelP = 0.0004;//0.0002
		public static final double kSpindexerWheelI = 0;
		public static final double kSpindexerWheelD = 0.01;
		public static final double kS = 0.18837;
		public static final double kV = 0.10525 / 60.0;

		public static final SimpleMotorFeedforward kFeedForward = new SimpleMotorFeedforward(kS, kV);
	}

	private final SparkFlex feederMotor;
	private final SparkFlexConfig feederMotorConfig = new SparkFlexConfig();
	private SysIdRoutine sysid = new SysIdRoutine(
		new SysIdRoutine.Config(Volts.per(Second).of(0.5), Volts.of(2), Seconds.of(5)),
		new SysIdRoutine.Mechanism(this::voltageDrive, this::logMotors, this)
	);

	public FeederSubsystem() {
		feederMotor = new SparkFlex(FeederConstants.kFeederMotorId, MotorType.kBrushless);

		feederMotorConfig.idleMode(IdleMode.kCoast);
		// intakeMotorConfig.smartCurrentLimit(80);
		feederMotorConfig.closedLoop
				.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
				.pid(FeederConstants.kSpindexerWheelP,
						FeederConstants.kSpindexerWheelI,
						FeederConstants.kSpindexerWheelD,
						ClosedLoopSlot.kSlot0)
				.outputRange(-0.75, 0.75, ClosedLoopSlot.kSlot0);

		

		feederMotor.configure(feederMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	
		SmartDashboard.putData("FeederSubsystem/QuasiStatic Forward", sysid.quasistatic(Direction.kForward));
		SmartDashboard.putData("FeederSubsystem/Quasistatic Backward", sysid.quasistatic(Direction.kReverse));
		SmartDashboard.putData("FeederSubsystem/Dynamic Forward", sysid.dynamic(Direction.kForward));
		SmartDashboard.putData("FeederSubsystem/Dynamic Backward", sysid.dynamic(Direction.kReverse));


	}

	public void spin(Boolean reverseMotor) {
		if (reverseMotor) {
			feederMotor.getClosedLoopController().setSetpoint(FeederConstants.kFeederSpeed, ControlType.kVelocity,
					ClosedLoopSlot.kSlot0, FeederConstants.kFeedForward.calculate(FeederConstants.kFeederSpeed));
		} else {
			feederMotor.getClosedLoopController().setSetpoint(-1 * FeederConstants.kFeederSpeed, ControlType.kVelocity,
					ClosedLoopSlot.kSlot0, FeederConstants.kFeedForward.calculate(-1 * FeederConstants.kFeederSpeed));
		}

	}

	public void stopMotor() {
		feederMotor.stopMotor();
	}

	/** Set motor voltage output. Used for sysid. */
	private void voltageDrive(Voltage volts) {
		feederMotor.setVoltage(volts);
	}

	/** Log motor encoder . */
	private void logMotors(SysIdRoutineLog log) {
		var encoder = feederMotor.getEncoder();
		log.motor("spindexer")
			.angularPosition(Rotations.of(encoder.getPosition()))
			.angularVelocity(Rotations.per(Minute).of(encoder.getVelocity()))
			.current(Amps.of(feederMotor.getOutputCurrent()))
			.voltage(Volts.of(feederMotor.getBusVoltage() * feederMotor.getAppliedOutput()));
	}

	/** Send dashboard data. */
	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Feeder Wheel Speed ", () -> feederMotor.getEncoder().getVelocity(), null);
	}

	@Override
	public void periodic() {
	}
}
