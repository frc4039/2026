package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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

public class ShooterSubsystem extends SubsystemBase {
	public final class ShooterConstants {
		// How fast the turret rotates from one point to another.
		public static final double KMotorVelocity = 20;

		// Device ids
		public static final int kLeaderMotorID = 32;
		public static final int kFollowerMotorID = 33;

		// Pid values
		public static final double kVShooter = 0.12341;
		public static final double kSShooter = 0.32361;
		// public static final double kAShooter = 0.010104;
		public static final double kAShooter = 0.0;
		public static final double kPShooter = 0.079764;
		public static final double kIShooter = 0.0;
		public static final double kDShooter = 0.0;

	}

	private TalonFX shooterLeaderMotor, shooterFollowerMotor;

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
					(volts) -> shooterLeaderMotor.setControl(voltRequest.withOutput(volts.in(Volts))),
					null,
					this));

	public ShooterSubsystem() {
		shooterLeaderMotor = new TalonFX(ShooterConstants.kLeaderMotorID);
		shooterFollowerMotor = new TalonFX(ShooterConstants.kFollowerMotorID);

		shooterLeaderMotor.setNeutralMode(NeutralModeValue.Coast);
		shooterFollowerMotor.setNeutralMode(NeutralModeValue.Coast);

		var follower = new Follower(ShooterConstants.kLeaderMotorID, MotorAlignmentValue.Opposed);

		shooterFollowerMotor.setControl(follower);
		MotorOutputConfigs mcfg = new MotorOutputConfigs();

		mcfg.withInverted(InvertedValue.CounterClockwise_Positive);
		mcfg.withNeutralMode(NeutralModeValue.Coast);

		TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration().withMotorOutput(mcfg);

		Slot0Configs slotConfigs = talonFXConfigs.Slot0;

		slotConfigs.kS = ShooterConstants.kSShooter;
		slotConfigs.kV = ShooterConstants.kVShooter;
		slotConfigs.kA = ShooterConstants.kAShooter;
		slotConfigs.kP = ShooterConstants.kPShooter;
		slotConfigs.kI = ShooterConstants.kIShooter;
		slotConfigs.kD = ShooterConstants.kDShooter;

		shooterLeaderMotor.getConfigurator().apply(talonFXConfigs);

		// SmartDashboard.putData("ShooterSubsystem/Start Logging", Commands.runOnce(SignalLogger::start));
		// SmartDashboard.putData("ShooterSubsystem/Stop Logging", Commands.runOnce(SignalLogger::stop));
		// SmartDashboard.putData("ShooterSubsystem/QuasiStatic Forward", sysid.quasistatic(Direction.kForward));
		// SmartDashboard.putData("ShooterSubsystem/Quasistatic Backward", sysid.quasistatic(Direction.kReverse));
		// SmartDashboard.putData("ShooterSubsystem/Dynamic Forward", sysid.dynamic(Direction.kForward));
		// SmartDashboard.putData("ShooterSubsystem/Dynamic Backward", sysid.dynamic(Direction.kReverse));

	}

	public void shoot(Boolean forward) {
		final VelocityVoltage request = new VelocityVoltage(ShooterConstants.KMotorVelocity).withSlot(0);

		if (forward) {
			shooterLeaderMotor.setControl(request.withVelocity(ShooterConstants.KMotorVelocity));
		} else {
			shooterLeaderMotor.setControl(request.withVelocity(-1 * ShooterConstants.KMotorVelocity));
		}
	}

	public void shootInput(double velocity) {
		final VelocityVoltage request = new VelocityVoltage(velocity).withSlot(0);
		shooterLeaderMotor.setControl(request.withVelocity(velocity));
	}

	public void manualVelocity() {
		final VelocityVoltage request = new VelocityVoltage(manualVelocity).withSlot(0);
		shooterLeaderMotor.setControl(request.withVelocity(manualVelocity));
	}

	public void stop() {
		shooterLeaderMotor.stopMotor();
	}

	public double getShooterError() {
		return shooterLeaderMotor.getClosedLoopError().getValueAsDouble();
	}

	@Override
	public void periodic() {
		// For safety, stop the turret whenever the robot is disabled.
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Shooter speed", () -> shooterLeaderMotor.getVelocity().getValueAsDouble(), null);
		builder.addDoubleProperty("Shooter acceleration", () -> shooterLeaderMotor.getAcceleration().getValueAsDouble(),
				null);
		builder.addDoubleProperty("Set Velocity", () -> manualVelocity, (manualVelocity) -> this.manualVelocity = manualVelocity);
	}

}
