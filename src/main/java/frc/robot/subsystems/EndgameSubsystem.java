package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndgameSubsystem extends SubsystemBase {
	public static final class EndgameConstants {
		static int kClimberMotorCanID = 42;
		static double kClimberSpeed = 0.25;

		static int kLimitSwitchId = 2;
	}

	private final SparkMax climberMotor;
	private final SparkMaxConfig climberMotorConfig = new SparkMaxConfig();
	private final DigitalInput limitSwitch = new DigitalInput(EndgameConstants.kLimitSwitchId);

	public EndgameSubsystem() {
		climberMotor = new SparkMax(EndgameConstants.kClimberMotorCanID, MotorType.kBrushless);

		climberMotorConfig.idleMode(IdleMode.kBrake);

		climberMotor.configure(climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	public void climb(boolean upwards) {
		if (upwards == false) {
			if (limitSwitch.get()) {
				climberMotor.set(0);
				climberMotor.getEncoder().setPosition(0);
			} else {
				climberMotor.set(-1 * EndgameConstants.kClimberSpeed);
			}
		}
		if (upwards == false) {
			climberMotor.getClosedLoopController().setSetpoint(100.0, ControlType.kPosition);
		}
	}

	public void stopMotor() {
		climberMotor.stopMotor();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Climber Position", () -> climberMotor.getEncoder().getPosition(), null);
	}
}
