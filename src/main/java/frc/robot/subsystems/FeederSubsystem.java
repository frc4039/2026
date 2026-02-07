package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

	private final TalonFX feederMotor;
	

	public FeederSubsystem(HardwareMonitor hardwareMonitor) {
		feederMotor = new TalonFX(SpindexerConstants.kSpindexerMotorId);

		feederMotor.setNeutralMode(NeutralModeValue.Coast);

		MotorOutputConfigs mcfg = new MotorOutputConfigs();

		mcfg.withInverted(InvertedValue.CounterClockwise_Positive);
		mcfg.withNeutralMode(NeutralModeValue.Coast);

		TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration().withMotorOutput(mcfg);

		Slot0Configs slotConfigs = talonFXConfigs.Slot0;

		slotConfigs.kS = FeederConstants.kS;
		slotConfigs.kV = SpindexerConstants.kV;
		// slotConfigs.kA = SpindexerConstants.kAShooter;
		slotConfigs.kP = FeederConstants.kTurretFeederWheelP;
		slotConfigs.kI = FeederConstants.kTurretFeederWheelI;
		slotConfigs.kD = FeederConstants.kTurretFeederWheelD;

		feederMotor.getConfigurator().apply(talonFXConfigs);

		hardwareMonitor.registerDevice(this, feederMotor);

	}

	public void feed(Boolean forward) {
		final VelocityVoltage request = new VelocityVoltage(FeederConstants.kTurretFeederSpeed).withSlot(0);

		if (forward) {
			feederMotor.setControl(request.withVelocity(FeederConstants.kTurretFeederSpeed));
		} else {
			feederMotor.setControl(request.withVelocity(-1 * FeederConstants.kTurretFeederSpeed));
		}
	}

	public void feedInput(double velocity) {
		final VelocityVoltage request = new VelocityVoltage(velocity).withSlot(0);
		feederMotor.setControl(request.withVelocity(velocity));
	}


	public void stop() {
		feederMotor.stopMotor();
	}

	@Override
	public void periodic() {
		// For safety, stop the turret whenever the robot is disabled.
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Feeder speed", () -> feederMotor.getVelocity().getValueAsDouble(), null);
	}

}
