package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.TurretSubsystem.TurretConstants;

public class ShooterAngleSubsystem extends SubsystemBase{
    
    public final class ShooterAngleConstants {
        public static final int motorId = 34;

        public static final double kSAngle = 0; // Overcome friction
        public static final double kVAngle = 0; // Velocity / Maxspeed
        public static final double kAAngle = 0; // Acceleration
        public static final double kPAngle = 0; //PID controller constants
        public static final double kIAngle = 0;
        public static final double kDAngle = 0; 

        public static final double kDegreesPerRotation = 257.142857142857142;

        public static final double kCruiseVelocity = 720.0 / kDegreesPerRotation;
		public static final double kAcceleration = 1440.0 / kDegreesPerRotation;
		public static final double kJerk = 10000.0 / kDegreesPerRotation;
    }

    private TalonFX angleMotor;

    public ShooterAngleSubsystem(){
        angleMotor = new TalonFX(ShooterAngleConstants.motorId);
        angleMotor.setNeutralMode(NeutralModeValue.Brake);

        MotorOutputConfigs mcfg = new MotorOutputConfigs();

		mcfg.withInverted(InvertedValue.Clockwise_Positive);
		mcfg.withNeutralMode(NeutralModeValue.Brake);

		TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration().withMotorOutput(mcfg);

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

		angleMotor.getConfigurator().apply(talonFXConfigs);
    }

    public void resetTurret(){
        angleMotor.setPosition(0);
    }

    public void moveToPosition(double position) {
		System.out.println(position);
		final MotionMagicVoltage request = new MotionMagicVoltage(position);

		final double newPosition = position / TurretConstants.kDegreesPerRotation;

		angleMotor.setControl(request.withPosition(newPosition)
				.withSlot(0)
				.withOverrideBrakeDurNeutral(true));
                
	}

    public double getTurretPosition() {
		return angleMotor.getPosition().getValueAsDouble() * TurretConstants.kDegreesPerRotation;
	}

}
