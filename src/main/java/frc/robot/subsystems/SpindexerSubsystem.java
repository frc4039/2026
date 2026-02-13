package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.utils.HardwareMonitor;
import frc.robot.utils.Helpers;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

public class SpindexerSubsystem extends SubsystemBase {
	// Constants of the feeder subsystem
	public static final class SpindexerConstants {
		public static final int kSpindexerMotorId = 41;
		public static final double kSpindexerSpeed = -1100;

		public static final double kSpindexerWheelP = 0.0004;//0.0002
		public static final double kSpindexerWheelI = 0;
		public static final double kSpindexerWheelD = 0.01;
		public static final double kS = 0.18837;
		public static final double kV = 0.10525 / 60.0;

		public static final SimpleMotorFeedforward kFeedForward = new SimpleMotorFeedforward(kS, kV);
	}

	private TalonFX spindexerMotor;
	private VoltageOut voltRequest = new VoltageOut(0.0);
	private SysIdRoutine sysid = new SysIdRoutine(
			new SysIdRoutine.Config(
				Volts.of(0.5).per(Second),
				Volts.of(4),
				Seconds.of(5),
				(state) -> SignalLogger.writeString("state", state.toString())
			),
			new SysIdRoutine.Mechanism(
					(volts) -> spindexerMotor.setControl(voltRequest.withOutput(volts.in(Volts))),
					null,
					this));

	public SpindexerSubsystem(HardwareMonitor hardwareMonitor) {
		spindexerMotor = new TalonFX(SpindexerConstants.kSpindexerMotorId);

		spindexerMotor.setNeutralMode(NeutralModeValue.Coast);

		MotorOutputConfigs mcfg = new MotorOutputConfigs();

		mcfg.withInverted(InvertedValue.CounterClockwise_Positive);
		mcfg.withNeutralMode(NeutralModeValue.Coast);

		TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration().withMotorOutput(mcfg);

		Slot0Configs slotConfigs = talonFXConfigs.Slot0;

		// slotConfigs.kS = SpindexerConstants.kSp;
		// slotConfigs.kV = SpindexerConstants.kVShooter;
		// slotConfigs.kA = SpindexerConstants.kAShooter;
		slotConfigs.kP = SpindexerConstants.kSpindexerWheelP;
		slotConfigs.kI = SpindexerConstants.kSpindexerWheelI;
		slotConfigs.kD = SpindexerConstants.kSpindexerWheelD;

		spindexerMotor.getConfigurator().apply(talonFXConfigs);

		hardwareMonitor.registerDevice(this, spindexerMotor);

		SmartDashboard.putData("SpindexerSubsystem/QuasiStatic Forward",sysid.quasistatic(Direction.kForward));
		SmartDashboard.putData("SpindexerSubsystem/QuasiStatic Backward",sysid.quasistatic(Direction.kReverse));
		SmartDashboard.putData("SpindexerSubsystem/Dynamic Forward",sysid.dynamic(Direction.kForward));
		SmartDashboard.putData("SpindexerSubsystem/Dynamic Backward",sysid.dynamic(Direction.kReverse));
	}

	public void spin(Boolean forward) {
		final VelocityVoltage request = new VelocityVoltage(SpindexerConstants.kSpindexerSpeed).withSlot(0);

		if (forward) {
			spindexerMotor.setControl(request.withVelocity(SpindexerConstants.kSpindexerSpeed));
		} else {
			spindexerMotor.setControl(request.withVelocity(-1 * SpindexerConstants.kSpindexerSpeed));
		}
	}

	public void spinInput(double velocity) {
		final VelocityVoltage request = new VelocityVoltage(velocity).withSlot(0);
		spindexerMotor.setControl(request.withVelocity(velocity));
	}


	public void stop() {
		spindexerMotor.stopMotor();
	}

	public void run() {
		spindexerMotor.set(-0.2);
	}

	@Override
	public void periodic() {
		// For safety, stop the turret whenever the robot is disabled.
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Spindexer speed", () -> spindexerMotor.getVelocity().getValueAsDouble(), null);
	}

}
