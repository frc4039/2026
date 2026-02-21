package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ManualVelocityCommand extends Command {
  private ShooterSubsystem shooterSubsystem;
  public ManualVelocityCommand(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooterSubsystem.manualVelocity();
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
