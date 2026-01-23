package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretFeederSubsystem extends SubsystemBase {
	public static final class TurretFeederConstants {
		static int kTurretFeederMotorId = 45;
		static double kTurretFeederSpeed = -0.5;
	}

	private final SparkFlex turretFeederMotor;
	private final SparkFlexConfig turretFeederMotorConfig = new SparkFlexConfig();

	public TurretFeederSubsystem() {
		turretFeederMotor = new SparkFlex(TurretFeederConstants.kTurretFeederMotorId, MotorType.kBrushless);

		turretFeederMotorConfig.idleMode(IdleMode.kCoast);

		turretFeederMotor.configure(turretFeederMotorConfig, ResetMode.kResetSafeParameters,
				PersistMode.kPersistParameters);
	}

	public void feed(Boolean reverseMotor) {
		if (reverseMotor) {
			turretFeederMotor.set(-1 * TurretFeederConstants.kTurretFeederSpeed);
		} else {
			turretFeederMotor.set(TurretFeederConstants.kTurretFeederSpeed);
		}
	}

	public void stopMotor() {
		turretFeederMotor.stopMotor();
	}

	@Override
	public void periodic() {

	}
}
