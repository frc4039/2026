package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretFeederSubsystem extends SubsystemBase {
	public static final class TurretFeederConstants {
		static int kTurretFeederMotorId = 45;
		static double kTurretFeederSpeed = 0.60;

	public static final double kTurretFeederWheelP = 0.5;
	public static final double kTurretFeederWheelI = 0;
    public static final double kTurretFeederWheelD = 0;
	public static final double kTurretFeederWheelFF = 0;

	}

	private final SparkFlex turretFeederMotor;
	private final SparkFlexConfig turretFeederMotorConfig = new SparkFlexConfig();

	public TurretFeederSubsystem() {
		turretFeederMotor = new SparkFlex(TurretFeederConstants.kTurretFeederMotorId, MotorType.kBrushless);

		turretFeederMotorConfig.idleMode(IdleMode.kCoast);
		// turretFeederMotorConfig.closedLoop
		// 	.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
		// 	.pid(TurretFeederConstants.kTurretFeederWheelP, 
		// 		TurretFeederConstants.kTurretFeederWheelI, 
		// 		TurretFeederConstants.kTurretFeederWheelD, 
		// 		ClosedLoopSlot.kSlot0)
		// 	.velocityFF(TurretFeederConstants.kTurretFeederWheelFF,ClosedLoopSlot.kSlot0)
		// 	.outputRange(-0.75, 0.75,ClosedLoopSlot.kSlot0);

		turretFeederMotor.configure(turretFeederMotorConfig, ResetMode.kResetSafeParameters,
				PersistMode.kPersistParameters);
	}

	public void feed(Boolean reverseMotor) {
		if (reverseMotor) {
			turretFeederMotor.set(TurretFeederConstants.kTurretFeederSpeed);		
		} else {
			turretFeederMotor.set(-1 * TurretFeederConstants.kTurretFeederSpeed);	
		}
	}

	public void stopMotor() {
		turretFeederMotor.stopMotor();
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Turret Feeder Wheel Speed: ", turretFeederMotor.getEncoder().getVelocity());
	}
}
