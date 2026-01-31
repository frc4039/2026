package frc.robot.subsystems;

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
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.SpindexerSubsystem.SpindexerConstants;
import frc.robot.utils.HardwareMonitor;;

public class FeederSubsystem extends SubsystemBase {
	public static final class FeederConstants {
		static int kTurretFeederMotorId = 45;
		static double kTurretFeederSpeed = 1500;

	public static final double kTurretFeederWheelP = 0.00009;
	public static final double kTurretFeederWheelI = 0.000003;
    public static final double kTurretFeederWheelD = 0.003;
	public static final double kTurretFeederWheelFF = 0;

	public static final double kS = 0.2355;
	public static final double kV = 0.11445 / 60.0;

	public static final SimpleMotorFeedforward kFeedForward = new SimpleMotorFeedforward(kS, kV);

	}

	private final SparkFlex feederMotor;
	private final SparkFlexConfig feederMotorConfig = new SparkFlexConfig();
	private SysIdRoutine sysid = new SysIdRoutine(
		new SysIdRoutine.Config(Volts.per(Second).of(0.5), Volts.of(2), Seconds.of(5)),
		new SysIdRoutine.Mechanism(this::voltageDrive, this::logMotors, this)
	);

	public FeederSubsystem(HardwareMonitor hardwareMonitor) {
		feederMotor = new SparkFlex(FeederConstants.kTurretFeederMotorId, MotorType.kBrushless);

		feederMotorConfig.idleMode(IdleMode.kCoast);
		feederMotorConfig.closedLoop
			.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
			.pid(FeederConstants.kTurretFeederWheelP, 
				FeederConstants.kTurretFeederWheelI, 
				FeederConstants.kTurretFeederWheelD, 
				ClosedLoopSlot.kSlot0)
			.outputRange(-1, 1,ClosedLoopSlot.kSlot0);

		feederMotor.configure(feederMotorConfig, ResetMode.kResetSafeParameters,
				PersistMode.kPersistParameters);

		hardwareMonitor.registerDevice(this, feederMotor);

		// SmartDashboard.putData("TurretFeederSubsystem/QuasiStatic Forward", sysid.quasistatic(Direction.kForward));
		// SmartDashboard.putData("TurretFeederSubsystem/Quasistatic Backward", sysid.quasistatic(Direction.kReverse));
		// SmartDashboard.putData("TurretFeederSubsystem/Dynamic Forward", sysid.dynamic(Direction.kForward));
		// SmartDashboard.putData("TurretFeederSubsystem/Dynamic Backward", sysid.dynamic(Direction.kReverse));

	}

	public void feed(Boolean reverseMotor) {
		if (reverseMotor) {
			feederMotor.getClosedLoopController().setSetpoint(FeederConstants.kTurretFeederSpeed, ControlType.kVelocity,
					ClosedLoopSlot.kSlot0, FeederConstants.kFeedForward.calculate(FeederConstants.kTurretFeederSpeed));
		} else {
			feederMotor.getClosedLoopController().setSetpoint(-1 * FeederConstants.kTurretFeederSpeed, ControlType.kVelocity,
					ClosedLoopSlot.kSlot0, FeederConstants.kFeedForward.calculate(-1 * FeederConstants.kTurretFeederSpeed));
		}
	}

	public void stopMotor() {
		feederMotor.stopMotor();
	}

	private void voltageDrive(Voltage volts) {
		feederMotor.setVoltage(volts);
	}

	private void logMotors(SysIdRoutineLog log) {
		var encoder = feederMotor.getEncoder();
		log.motor("turret feeder motor")
			.angularPosition(Rotations.of(encoder.getPosition()))
			.angularVelocity(Rotations.per(Minute).of(encoder.getVelocity()))
			.current(Amps.of(feederMotor.getOutputCurrent()))
			.voltage(Volts.of(feederMotor.getBusVoltage() * feederMotor.getAppliedOutput()));
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Feeder Wheel Speed", () -> feederMotor.getEncoder().getVelocity(), null);
		builder.addDoubleProperty("Current", () -> feederMotor.getOutputCurrent(), null);
	}

	public void periodic() {}
}
