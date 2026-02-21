
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class ResetTurretGyro extends Command {
  private TurretSubsystem turretSubsystem;
  public ResetTurretGyro(TurretSubsystem turretSubsystem) {
    this.turretSubsystem = turretSubsystem;
    addRequirements(turretSubsystem);
  }

  @Override
  public void initialize() {

    turretSubsystem.resetTurret();
	}

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
