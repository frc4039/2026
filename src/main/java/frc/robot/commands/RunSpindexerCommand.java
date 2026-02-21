
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SpindexerSubsystem;

public class RunSpindexerCommand extends Command {
  private SpindexerSubsystem spindexerSubsystem;
  public RunSpindexerCommand(SpindexerSubsystem spindexerSubsystem) {
    this.spindexerSubsystem = spindexerSubsystem;
    addRequirements(spindexerSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    spindexerSubsystem.run();
  }

  @Override
  public void end(boolean interrupted) {
    spindexerSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
