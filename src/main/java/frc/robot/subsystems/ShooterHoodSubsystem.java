package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.TurretSubsystem.TurretConstants;

public class ShooterHoodSubsystem extends SubsystemBase{
    
    public final class ShooterAngleConstants {
        public static final int motorId = 34;

        public static final double kSAngle = 0; // Overcome friction
        public static final double kVAngle = 0; // Velocity / Maxspeed
        public static final double kAAngle = 0; // Acceleration
        public static final double kPAngle = 1; //PID controller constants
        public static final double kIAngle = 0;
        public static final double kDAngle = 0; 

		public static final double gearRatio = (4.0 * 4.0) * (15.0/14.0) * (180.0/15.0);
        public static final double kDegreesPerRotation = 360.0 / gearRatio;

        public static final double kCruiseVelocity = 720.0 / kDegreesPerRotation;
		public static final double kAcceleration = 1440.0 / kDegreesPerRotation;
		public static final double kJerk = 10000.0 / kDegreesPerRotation;

		public static final double angleThreshold = 0.5;

		public static final double kHoodOffset = 75;

		//Hard limits for hood angle
		public static final double kMin = 45;
		public static final double kMax = 76;
    }

    private TalonFX hoodMotor;

    public ShooterHoodSubsystem(){
        hoodMotor = new TalonFX(ShooterAngleConstants.motorId);
        hoodMotor.setNeutralMode(NeutralModeValue.Coast);
		

        MotorOutputConfigs mcfg = new MotorOutputConfigs();

		mcfg.withInverted(InvertedValue.CounterClockwise_Positive);
		mcfg.withNeutralMode(NeutralModeValue.Coast);

		TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration().withMotorOutput(mcfg);
		talonFXConfigs.CurrentLimits.StatorCurrentLimit = 15;
		talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
		Slot0Configs slotConfigs = talonFXConfigs.Slot0;

		slotConfigs.kS = ShooterAngleConstants.kSAngle;
		slotConfigs.kV = ShooterAngleConstants.kVAngle;
		slotConfigs.kA = ShooterAngleConstants.kAAngle;     
		slotConfigs.kP = ShooterAngleConstants.kPAngle;
		slotConfigs.kI = ShooterAngleConstants.kIAngle;
		slotConfigs.kD = ShooterAngleConstants.kDAngle;

		MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;

		motionMagicConfigs.MotionMagicCruiseVelocity = ShooterAngleConstants.kCruiseVelocity;
		motionMagicConfigs.MotionMagicAcceleration = ShooterAngleConstants.kAcceleration;
		motionMagicConfigs.MotionMagicJerk = ShooterAngleConstants.kJerk;

		hoodMotor.getConfigurator().apply(talonFXConfigs);
    }

    public void resetTurret(){
        hoodMotor.setPosition(ShooterAngleConstants.kHoodOffset / ShooterAngleConstants.kDegreesPerRotation);
    }

    public void moveToPosition(double position) {
		final double newPosition = position / ShooterAngleConstants.kDegreesPerRotation;
		final MotionMagicVoltage request = new MotionMagicVoltage(newPosition);


		hoodMotor.setControl(request.withPosition(newPosition)
				.withSlot(0)
				.withOverrideBrakeDurNeutral(false));
                
	}

    public double getHoodPosition() {
		return hoodMotor.getPosition().getValueAsDouble() * ShooterAngleConstants.kDegreesPerRotation;
	}

	public void stopMotor() {
		hoodMotor.stopMotor();
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Hood Angle", () -> this.getHoodPosition(), position -> this.moveToPosition(position));
		builder.addDoubleProperty("Current", () -> hoodMotor.getStatorCurrent().getValueAsDouble(), null);
	}
}
