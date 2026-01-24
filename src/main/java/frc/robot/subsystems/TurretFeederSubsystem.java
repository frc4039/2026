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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.FeederSubsystem.FeederConstants;

public class TurretFeederSubsystem extends SubsystemBase {
	public static final class TurretFeederConstants {
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

	private final SparkFlex turretFeederMotor;
	private final SparkFlexConfig turretFeederMotorConfig = new SparkFlexConfig();
	private SysIdRoutine sysid = new SysIdRoutine(
		new SysIdRoutine.Config(Volts.per(Second).of(0.5), Volts.of(2), Seconds.of(5)),
		new SysIdRoutine.Mechanism(this::voltageDrive, this::logMotors, this)
	);

	public TurretFeederSubsystem() {
		turretFeederMotor = new SparkFlex(TurretFeederConstants.kTurretFeederMotorId, MotorType.kBrushless);

		turretFeederMotorConfig.idleMode(IdleMode.kCoast);
		turretFeederMotorConfig.closedLoop
			.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
			.pid(TurretFeederConstants.kTurretFeederWheelP, 
				TurretFeederConstants.kTurretFeederWheelI, 
				TurretFeederConstants.kTurretFeederWheelD, 
				ClosedLoopSlot.kSlot0)
			.outputRange(-1, 1,ClosedLoopSlot.kSlot0);

		turretFeederMotor.configure(turretFeederMotorConfig, ResetMode.kResetSafeParameters,
				PersistMode.kPersistParameters);

		// SmartDashboard.putData("TurretFeederSubsystem/QuasiStatic Forward", sysid.quasistatic(Direction.kForward));
		// SmartDashboard.putData("TurretFeederSubsystem/Quasistatic Backward", sysid.quasistatic(Direction.kReverse));
		// SmartDashboard.putData("TurretFeederSubsystem/Dynamic Forward", sysid.dynamic(Direction.kForward));
		// SmartDashboard.putData("TurretFeederSubsystem/Dynamic Backward", sysid.dynamic(Direction.kReverse));

	}

	public void feed(Boolean reverseMotor) {
		if (reverseMotor) {
			turretFeederMotor.getClosedLoopController().setSetpoint(TurretFeederConstants.kTurretFeederSpeed, ControlType.kVelocity,
					ClosedLoopSlot.kSlot0, TurretFeederConstants.kFeedForward.calculate(TurretFeederConstants.kTurretFeederSpeed));
		} else {
			turretFeederMotor.getClosedLoopController().setSetpoint(-1 * TurretFeederConstants.kTurretFeederSpeed, ControlType.kVelocity,
					ClosedLoopSlot.kSlot0, TurretFeederConstants.kFeedForward.calculate(-1 * TurretFeederConstants.kTurretFeederSpeed));
		}
	}

	public void stopMotor() {
		turretFeederMotor.stopMotor();
	}

	private void voltageDrive(Voltage volts) {
		turretFeederMotor.setVoltage(volts);
	}

	private void logMotors(SysIdRoutineLog log) {
		var encoder = turretFeederMotor.getEncoder();
		log.motor("turret feeder motor")
			.angularPosition(Rotations.of(encoder.getPosition()))
			.angularVelocity(Rotations.per(Minute).of(encoder.getVelocity()))
			.current(Amps.of(turretFeederMotor.getOutputCurrent()))
			.voltage(Volts.of(turretFeederMotor.getBusVoltage() * turretFeederMotor.getAppliedOutput()));
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Turret Feeder Wheel Speed:", turretFeederMotor.getEncoder().getVelocity());
	}
}
