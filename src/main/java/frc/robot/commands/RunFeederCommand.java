
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;


public class RunFeederCommand extends Command {
  private FeederSubsystem feederSubsystem;
  public RunFeederCommand(FeederSubsystem feederSubsystem) {
    this.feederSubsystem = feederSubsystem;
    addRequirements(feederSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    feederSubsystem.run();
  }

  @Override
  public void end(boolean interrupted) {
    feederSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
