package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.utils.HardwareMonitor;

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


public class SpindexerSubsystem extends SubsystemBase {
	// Constants of the feeder subsystem
	public static final class SpindexerConstants {
		public static final int kSpindexerMotorId = 41;
		
		// RPS: gear ratio 3:!
		public static final double kSpindexerSpeed = -200;

		//FeedForward and PID Constants
		public static final double kSpindexerWheelP = 0.021206;
		public static final double kSpindexerWheelI = 0.0;
		public static final double kSpindexerWheelD = 0.0;
		public static final double kS = 0.32889;
		public static final double kV = 0.10262;
		public static final double kA = 0.0025405;

		public static final SimpleMotorFeedforward kFeedForward = new SimpleMotorFeedforward(kS, kV);
	}

	private TalonFX spindexerMotor;
	private VoltageOut voltRequest = new VoltageOut(0.0);
	//Test to get ideal feed forward values
	/* 
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
	*/
	public SpindexerSubsystem(HardwareMonitor hardwareMonitor) {
		//Defines the motor and configs it
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

		//SmartDashboard.putData("SpindexerSubsystem/QuasiStatic Forward",sysid.quasistatic(Direction.kForward));
		//SmartDashboard.putData("SpindexerSubsystem/QuasiStatic Backward",sysid.quasistatic(Direction.kReverse));
		//SmartDashboard.putData("SpindexerSubsystem/Dynamic Forward",sysid.dynamic(Direction.kForward));
		//SmartDashboard.putData("SpindexerSubsystem/Dynamic Backward",sysid.dynamic(Direction.kReverse));
		SmartDashboard.putData("ShooterSubsystem/Start Logging", Commands.runOnce(SignalLogger::start));
		SmartDashboard.putData("ShooterSubsystem/Stop Logging", Commands.runOnce(SignalLogger::stop));
		

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
		//Sends Spindexers speed to elastic 
		builder.addDoubleProperty("Spindexer speed", () -> spindexerMotor.getVelocity().getValueAsDouble(), null);
	}

}
