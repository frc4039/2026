package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.ShooterSubsystem.ShooterConstants;

public class TurretSubsystem extends SubsystemBase {
	public final class TurretConstants {
		// How fast the turret rotates from one point to another.
		public static final double kRotationSpeed = 1.0;

		// Device id
		public static final int kMotorID = 31;

		// Gear Stuff
		public static final double kTurretPulleyTeeth = 120.0;
		public static final double kMotorPulleyTeeth = 23.0;
		public static final double kGearRatio = kTurretPulleyTeeth / kMotorPulleyTeeth;

		// Encoder Conversions;
		public static final double kTickersPerRotation = 4096;
		public static final double kDegreesPerRotation = 360.0 / kGearRatio;

		// Pid values
		public static final double kSTurret = 0.0;
		public static final double kVTurret = 0.0;
		public static final double kATurret = 0.1;
		public static final double kPTurret = 60.0;
		public static final double kITurret = 1.0;
		public static final double kDTurret = 0.0;

		// Motion magic
		public static final double kCruiseVelocity = 1500.0 / kDegreesPerRotation;
		public static final double kAcceleration = 2000.0 / kDegreesPerRotation;
		public static final double kJerk = 10000.0 / kDegreesPerRotation;

		// The accuracy (in degrees) that the turret has to be from it's target position
		// for the command to be considered done.
		public static final double kTurretAccuracyThreshold = 1.0;

		// Positions
		public static final double kTurretForwardPosition = 180.0;
		public static final double kTurretBackwardPosition = 360.0;

		// Turret Offset
		public static final Transform2d kTurretOffset = new Transform2d(Units.inchesToMeters(5.75),
				Units.inchesToMeters(5.25), Rotation2d.fromDegrees(180));

		// Min/Max
		public static final double kMin = -180;
		public static final double kMax = 50;

		public static final double kHubTargetHeight = 1.4351;
		public static final double kTurretHeight = 0.5;
		public static final double kDeltaZ = kHubTargetHeight - kTurretHeight;
		public static final double kVelocityZ = 7.3;
		public static final double kTimeOfFlight = (kVelocityZ
				+ Math.sqrt(Math.pow(kVelocityZ, 2) - (2 * 9.81 * kDeltaZ))) / 9.81;
		public static final double kShooterWheelCircumference = Units.inchesToMeters(4 * Math.PI);
	}

	private TalonFX turretMotor;
	private DriveSubsystem driveSubsystem;

	public TurretSubsystem(DriveSubsystem driveSubsystem) {
		turretMotor = new TalonFX(TurretConstants.kMotorID);
		this.driveSubsystem = driveSubsystem;

		turretMotor.setNeutralMode(NeutralModeValue.Brake);

		MotorOutputConfigs mcfg = new MotorOutputConfigs();

		mcfg.withInverted(InvertedValue.Clockwise_Positive);
		mcfg.withNeutralMode(NeutralModeValue.Brake);

		TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration().withMotorOutput(mcfg);

		Slot0Configs slotConfigs = talonFXConfigs.Slot0;

		slotConfigs.kS = TurretConstants.kSTurret;
		slotConfigs.kV = TurretConstants.kVTurret;
		slotConfigs.kA = TurretConstants.kATurret;
		slotConfigs.kP = TurretConstants.kPTurret;
		slotConfigs.kI = TurretConstants.kITurret;
		slotConfigs.kD = TurretConstants.kDTurret;

		MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;

		motionMagicConfigs.MotionMagicCruiseVelocity = TurretConstants.kCruiseVelocity;
		motionMagicConfigs.MotionMagicAcceleration = TurretConstants.kAcceleration;
		motionMagicConfigs.MotionMagicJerk = TurretConstants.kJerk;

		turretMotor.getConfigurator().apply(talonFXConfigs);
	}

	public void moveToPosition(double position) {
		final MotionMagicVoltage request = new MotionMagicVoltage(position);

		final double newPosition = position / TurretConstants.kDegreesPerRotation;

		turretMotor.setControl(request.withPosition(newPosition)
				.withSlot(0)
				.withOverrideBrakeDurNeutral(true));
	}

	public void resetTurret() {
		turretMotor.setPosition(0.0 / TurretConstants.kDegreesPerRotation);
	}

	public double getTurretPosition() {
		return turretMotor.getPosition().getValueAsDouble() * TurretConstants.kDegreesPerRotation;
	}

	public void moveToOwlHeadPosition(DoubleSupplier robotHeading, double desiredDirection) {
		double owlHeadPosition = robotHeading.getAsDouble() + desiredDirection;
		this.moveToPosition((owlHeadPosition % 360 + 360) % 360);
	}

	public static Pose2d getHub() {
		Optional<Alliance> alliance = DriverStation.getAlliance();
		if (alliance.isPresent()) {
			if (alliance.get() == Alliance.Red) {
				return FieldConstants.kRedHub;
			} else {
				return FieldConstants.kBlueHub;
			}
		}
		return FieldConstants.kRedHub;
	}

	public void stopMotor() {
		turretMotor.stopMotor();
		System.out.println("Trying to disable turret motor");
	}

	public double getXVelocity() {
		return driveSubsystem.getDistanceFromHub() / TurretConstants.kTimeOfFlight;
	}

	public double getOutputVelocity() {
		return Math.sqrt(Math.pow(this.getXVelocity(), 2) + Math.pow(TurretConstants.kVelocityZ, 2));
	}

	public double getHoodAngle() {
		return Units.radiansToDegrees(Math.atan2(TurretConstants.kVelocityZ, getXVelocity()));
	}

	public Pose2d getTurretPose() {
		return driveSubsystem.getPose()
				.plus(TurretConstants.kTurretOffset)
				.plus(new Transform2d(
						new Translation2d(),
						Rotation2d.fromDegrees(-1 * this.getTurretPosition())));
	}

	@Override
	public void periodic() {
		// For safety, stop the turret whenever the robot is disabled.
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Turret encoder degrees", () -> getTurretPosition(), null);
		builder.addDoubleProperty("Turret speed", () -> turretMotor.getVelocity().getValueAsDouble(), null);
		builder.addDoubleProperty("Turret acceleration", () -> turretMotor.getAcceleration().getValueAsDouble(), null);
		builder.addDoubleProperty("Turret Target", () -> {
			var controlInfo = turretMotor.getAppliedControl();
			if(controlInfo instanceof MotionMagicVoltage){
				return ((MotionMagicVoltage) controlInfo).Position * TurretConstants.kDegreesPerRotation;
			}
			return 0;
		}, null);
		builder.addDoubleProperty("Shooter Target", () -> this.getOutputVelocity(), null);
		builder.addDoubleProperty("Target", () -> driveSubsystem.getDistanceFromHub(), null);
		// builder.addDoubleProperty("Turret Target",() -> {
		// moveToPosition(Math.min(TurretConstants.kMax, Math.max(TurretConstants.kMin,
		// Pose2d currentRobotPose2d = driveSubsystem.getPose().plus(TurretConstants.kTurretOffset);
		// Pose2d hubPose2d = TurretSubsystem.getHub();
		// 		-1 * hubPose2d.relativeTo(currentRobotPose2d).getTranslation().getAngle().getDegrees())))
		// }, null);
	}
}
