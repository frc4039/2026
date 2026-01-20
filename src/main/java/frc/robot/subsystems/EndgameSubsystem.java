package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndgameSubsystem extends SubsystemBase {
	public static final class EndgameConstants {
		static int kClimberMotorCanID = 42;
		static double kClimberSpeed = 0.25;
	}

	private final SparkMax climberMotor;
	private final SparkMaxConfig climberMotorConfig = new SparkMaxConfig();

	public EndgameSubsystem() {
		climberMotor = new SparkMax(EndgameConstants.kClimberMotorCanID, MotorType.kBrushless);

		climberMotorConfig.idleMode(IdleMode.kBrake);

		climberMotor.configure(climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	public void climb(boolean upwards) {
		if (upwards) {
			climberMotor.set(EndgameConstants.kClimberSpeed);
		} else {
			climberMotor.set(-1 * EndgameConstants.kClimberSpeed);
		}
	}

	public void stopMotor() {
		climberMotor.stopMotor();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
