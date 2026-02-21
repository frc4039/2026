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
import frc.robot.utils.HardwareMonitor;


public class TurretSubsystem extends SubsystemBase {
	public final class TurretConstants {
		// How fast the turret rotates from one point to another.
		public static final double kRotationSpeed = 1.0;

		// Device id
		public static final int kMotorID = 31;

		// Gear Stuff
		public static final double kTurretPulleyTeeth = 120.0;
		public static final double kMotorPulleyTeeth = 24.0;
		public static final double kGearRatio = (kTurretPulleyTeeth / kMotorPulleyTeeth) * 4;

		// Encoder Conversions;
		public static final double kTickersPerRotation = 4096;
		public static final double kDegreesPerRotation = 360.0 / kGearRatio;

		// Pid values
		public static final double kSTurret = 0.2673;
		public static final double kVTurret = 0.05462;
		public static final double kATurret = 0.0052418;
		public static final double kPTurret = 18.192;
		public static final double kITurret = 0.0;
		public static final double kDTurret = 0.05;
		public static final double kGTurret = -2.0;

		// Motion magic
		public static final double kCruiseVelocity = 30.0;
		public static final double kAcceleration = 250.0;
		public static final double kJerk = 1600.0;

		// The accuracy (in degrees) that the turret has to be from it's target position
		// for the command to be considered done.
		public static final double kTurretAccuracyThreshold = 1.0;

		// Positions
		public static final double kTurretForwardPosition = 180.0;
		public static final double kTurretBackwardPosition = 360.0;

		// Turret Offset
		public static final Transform2d kTurretOffset = new Transform2d(Units.inchesToMeters(-5.75),
				Units.inchesToMeters(-5.25), Rotation2d.fromDegrees(180));

		// Min/Max
		public static final double kMin = -180;
		public static final double kMax = 180;

		//Hub Height for the Projectile motion
		public static final double kHubTargetHeight = 1.4351;

		//Turret Height for Math
		public static final double kTurretHeight = 0.5;

		//Difference between the turret and hub on the vertical axis
		public static final double kDeltaZ = kHubTargetHeight - kTurretHeight;

		//Velocity on the up direction
		public static final double kVelocityZ = 7.3;

		//Calculations for time of flight based on the z Veloctiy and deltaZ
		public static final double kTimeOfFlight = (kVelocityZ
				+ Math.sqrt(Math.pow(kVelocityZ, 2) - (2 * 9.81 * kDeltaZ))) / 9.81;

		//Circumference of the shooter wheel
		public static final double kShooterWheelCircumference = Units.inchesToMeters(4 * Math.PI);

		//Maximum amount the Operator can offset the hub also how much each offset is by
		public static final double kManualChangeLimit = Units.inchesToMeters(18);
		public static final double kManualChangeAmount = Units.inchesToMeters(3);

		//Acounts for offset when driving quickly
		public static final double kLatencyOffset = 0.1;
	}

	public static enum AimState {
		AUTOMATIC,
		LEFT,
		RIGHT
	};

	private TalonFX turretMotor;
	private DriveSubsystem driveSubsystem;
	public static double xTransform = 0;
	public static double yTransform = 0;

	
	// private VoltageOut voltRequest = new VoltageOut(0.0);
	// private SysIdRoutine sysid = new SysIdRoutine(
	// 		new SysIdRoutine.Config(
	// 			Volts.of(1.0).per(Second),
	// 			Volts.of(5),
	// 			Seconds.of(5),
	// 			(state) -> SignalLogger.writeString("state", state.toString())
	// 		),
	// 		new SysIdRoutine.Mechanism(
	// 				(volts) -> turretMotor.setControl(voltRequest.withOutput(volts.in(Volts))),
	// 				null,
	// 				this));
	

	public TurretSubsystem(DriveSubsystem driveSubsystem, HardwareMonitor hardwareMonitor) {
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
		slotConfigs.kG = TurretConstants.kGTurret;

		MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;

		motionMagicConfigs.MotionMagicCruiseVelocity = TurretConstants.kCruiseVelocity;
		motionMagicConfigs.MotionMagicAcceleration = TurretConstants.kAcceleration;
		motionMagicConfigs.MotionMagicJerk = TurretConstants.kJerk;

		turretMotor.getConfigurator().apply(talonFXConfigs);

		hardwareMonitor.registerDevice(this, turretMotor);


		// SmartDashboard.putData("Turret Subsystem/Start Logging", Commands.runOnce(SignalLogger::start));
		// SmartDashboard.putData("Turret Subsystem/Stop Logging", Commands.runOnce(SignalLogger::stop));
		// SmartDashboard.putData("Turret Subsystem/QuasiStatic Forward", sysid.quasistatic(Direction.kForward));
		// SmartDashboard.putData("Turret Subsystem/Quasistatic Backward", sysid.quasistatic(Direction.kReverse));
		// SmartDashboard.putData("Turret Subsystem/Dynamic Forward", sysid.dynamic(Direction.kForward));
		// SmartDashboard.putData("Turret Subsystem/Dynamic Backward", sysid.dynamic(Direction.kReverse));
	}

	//Moves to a given position
	public void moveToPosition(double position) {
		final MotionMagicVoltage request = new MotionMagicVoltage(position);

		final double newPosition = position / TurretConstants.kDegreesPerRotation;

		turretMotor.setControl(request.withPosition(newPosition)
				.withSlot(0)
				.withOverrideBrakeDurNeutral(true));
	}

	//Resets the turret to zero
	public void resetTurret() {
		turretMotor.setPosition(0.0 / TurretConstants.kDegreesPerRotation);
	}

	//Returns the position of the turret
	public double getTurretPosition() {
		return turretMotor.getPosition().getValueAsDouble() * TurretConstants.kDegreesPerRotation;
	}

	public void moveToOwlHeadPosition(DoubleSupplier robotHeading, double desiredDirection) {
		double owlHeadPosition = robotHeading.getAsDouble() + desiredDirection;
		this.moveToPosition((owlHeadPosition % 360 + 360) % 360);
	}

	//Command for operator to change the location of the hub
	public static Transform2d changeTargetLocation(String direction) {
		Optional<Alliance> alliance = DriverStation.getAlliance();
		
		if(direction == "up") {
			xTransform += -1 * TurretConstants.kManualChangeAmount;
		} else if (direction == "down") {
			xTransform += TurretConstants.kManualChangeAmount;
		} else if(direction == "left") {
			yTransform += -1 * TurretConstants.kManualChangeAmount;
		} else if(direction == "right") {
			yTransform += TurretConstants.kManualChangeAmount;
		}

		if(xTransform >= TurretConstants.kManualChangeLimit) {
			xTransform = TurretConstants.kManualChangeLimit;
		} if(xTransform <= -1 * TurretConstants.kManualChangeLimit) {
			xTransform = -1 * TurretConstants.kManualChangeLimit;
		} if(yTransform >= TurretConstants.kManualChangeLimit) {
			yTransform = TurretConstants.kManualChangeLimit;
		} if(yTransform <= -1 * TurretConstants.kManualChangeLimit) {
			yTransform = -1 * TurretConstants.kManualChangeLimit;
		} 
		
		
		if(alliance.isPresent()) {
		if(alliance.get() == Alliance.Red) {
			return new Transform2d(xTransform, yTransform, new Rotation2d(0));
		} else {
			return new Transform2d(-1 * xTransform, -1 * yTransform, new Rotation2d(0));
		}
		} else {
			return new Transform2d(xTransform, yTransform, new Rotation2d(0));
		}
		
	}

	
	//Returns the pose of the hub
	public static Pose2d getHub() {
		Optional<Alliance> alliance = DriverStation.getAlliance();
		if (alliance.isPresent()) {
			if (alliance.get() == Alliance.Red) {
				return FieldConstants.kRedHub.plus(changeTargetLocation("67"));
			} else {
				return FieldConstants.kBlueHub.plus(changeTargetLocation("67"));
			}
		}
		return FieldConstants.kRedHub.plus(changeTargetLocation("67"));
	}

	//Stops the turret motor
	public void stopMotor() {
		turretMotor.stopMotor();
		System.out.println("Trying to disable turret motor");
	}

	//Velocity needed in the x direction to reach the hub
	public double getXVelocity() {
		return driveSubsystem.getDistanceFromHub() / TurretConstants.kTimeOfFlight;
	}

	//Returns the output velocity for the wheels
	public double getOutputVelocity() {
		return Math.sqrt(Math.pow(this.getXVelocity(), 2) + Math.pow(TurretConstants.kVelocityZ, 2));
	}

	//Returns the angle the hood needs to hit a shot
	public double getHoodAngle() {
		return Units.radiansToDegrees(Math.atan2(TurretConstants.kVelocityZ, getXVelocity()));
	}

	//Returns the Pose of the turret
	public Pose2d getTurretPose() {
		return driveSubsystem.getPose()
				.plus(TurretConstants.kTurretOffset)
				.plus(new Transform2d(
						new Translation2d(),
						Rotation2d.fromDegrees(-1 * this.getTurretPosition())));
	}

	//Runs the turret at a set power
	public void runTurretPercentage(double power) {
		turretMotor.set(power);
	}

	//Returns the difference between the Turret's goal and its current position
	public double getTurretError() {
		return turretMotor.getClosedLoopError().getValueAsDouble();
	}

	public boolean isAtPosition() {
		return (Math.abs((this.getTurretPosition()) - (turretMotor.getClosedLoopReference().getValueAsDouble())) < 2);
	}

	@Override
	public void periodic() {
		// For safety, stop the turret whenever the robot is disabled.
	}

	//Data for the Dashboard
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
		builder.addDoubleProperty("Turret Voltage", () -> turretMotor.getMotorVoltage().getValueAsDouble(), null);
		builder.addDoubleProperty("Turret Error", () -> turretMotor.getClosedLoopError().getValueAsDouble() /  TurretConstants.kDegreesPerRotation, null);
		// builder.addDoubleProperty("Turret Target",() -> {
		// moveToPosition(Math.min(TurretConstants.kMax, Math.max(TurretConstants.kMin,
		// Pose2d currentRobotPose2d = driveSubsystem.getPose().plus(TurretConstants.kTurretOffset);
		// Pose2d hubPose2d = TurretSubsystem.getHub();
		// 		-1 * hubPose2d.relativeTo(currentRobotPose2d).getTranslation().getAngle().getDegrees())))
		// }, null);
	}
}
