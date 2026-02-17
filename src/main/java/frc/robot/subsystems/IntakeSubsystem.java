// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.HardwareMonitor;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeSubsystem extends SubsystemBase {
	/** Creates a new Intake. */
	public static final class IntakeContants {
		static int kIntakeMotorCanID = 21;
		static double kIntakeSpeed = -0.6;
		static double kOutakeSpeed = 1;
	}

	private final TalonFX intakeMotor;
	private final TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();

	public IntakeSubsystem(HardwareMonitor hardwareMonitor) {
		intakeMotor = new TalonFX(IntakeSubsystem.IntakeContants.kIntakeMotorCanID);

		MotorOutputConfigs mcfg = new MotorOutputConfigs();
		mcfg.withNeutralMode(NeutralModeValue.Brake);
		intakeMotorConfig.withMotorOutput(mcfg);
		intakeMotor.getConfigurator().apply(intakeMotorConfig);
		hardwareMonitor.registerDevice(this, intakeMotor);
	}

	public void intake() {
		// intakeMotor.getClosedLoopController().setSetpoint(IntakeSubsystem.IntakeContants.kIntakeSpeed,
		// ControlType.kVelocity);
		intakeMotor.set(IntakeSubsystem.IntakeContants.kIntakeSpeed);
	}

	public void outtake() {
		// intakeMotor.getClosedLoopController().setSetpoint(-1 *
		// IntakeSubsystem.IntakeContants.kIntakeSpeed, ControlType.kVelocity);
		intakeMotor.set(IntakeSubsystem.IntakeContants.kOutakeSpeed);
	}

	public void stopMotor() {
		intakeMotor.stopMotor();
	}

	public void periodic() {
		// This method will be called once per scheduler run
	}
}
