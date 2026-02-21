// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;


public class RunTurretPowerCommand extends Command {
  private TurretSubsystem turretSubsystem;
  private DoubleSupplier power;
  public RunTurretPowerCommand(TurretSubsystem turretSubsystem, DoubleSupplier power) {
    this.turretSubsystem = turretSubsystem;
    this.power = power;
    addRequirements(turretSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    turretSubsystem.runTurretPercentage((power.getAsDouble() - 0.25) / 4.0);
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
