// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */
  public static final class IntakeContants {
    static int kIntakeMotorCanID = 13;
    static double kIntakeSpeed = 200;
  }

  private final SparkFlex intakeMotor;
  private final SparkFlexConfig intakeMotorConfig = new SparkFlexConfig();

  public IntakeSubsystem() {
    intakeMotor = new SparkFlex(IntakeSubsystem.IntakeContants.kIntakeMotorCanID, MotorType.kBrushless);

    intakeMotorConfig.idleMode(IdleMode.kBrake);
    intakeMotorConfig.smartCurrentLimit(40);

    intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void intake(){
    intakeMotor.getClosedLoopController().setSetpoint(IntakeSubsystem.IntakeContants.kIntakeSpeed, ControlType.kVelocity);
  }

  public void stopMotor() {
    intakeMotor.stopMotor();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
