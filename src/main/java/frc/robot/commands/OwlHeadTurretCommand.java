package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class OwlHeadTurretCommand extends Command {
  private DoubleSupplier robotHeading;
  private final TurretSubsystem turretSubsystem;
  public OwlHeadTurretCommand(DoubleSupplier robotHeading, TurretSubsystem turretSubsystem) {
    this.robotHeading = robotHeading;
    this.turretSubsystem = turretSubsystem;
    addRequirements(turretSubsystem);

  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    turretSubsystem.moveToPosition(this.robotHeading.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
