package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class FeederSubsystem extends SubsystemBase {
	// Constants of the feeder subsystem
	public static final class FeederConstants {
		static int kFeederMotorId = 41;
		static double kFeederSpeed = 0.5;
	}

	private final SparkFlex feederMotor;
	private final SparkFlexConfig feederMotorConfig = new SparkFlexConfig();

	public FeederSubsystem() {
		feederMotor = new SparkFlex(IntakeSubsystem.IntakeContants.kIntakeMotorCanID, MotorType.kBrushless);

		feederMotorConfig.idleMode(IdleMode.kBrake);
		// intakeMotorConfig.smartCurrentLimit(80);

		feederMotor.configure(feederMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

	}

	public void spin(Boolean reverseMotor) {
		if (reverseMotor) {
			feederMotor.set(-1 * IntakeSubsystem.IntakeContants.kIntakeSpeed);
		} else {
			feederMotor.set(IntakeSubsystem.IntakeContants.kIntakeSpeed);
		}
	}

	public void stopMotor() {
		feederMotor.stopMotor();
	}

	@Override
	public void periodic() {
	}
}
