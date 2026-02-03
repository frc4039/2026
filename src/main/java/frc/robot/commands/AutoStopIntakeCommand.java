package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoStopIntakeCommand extends InstantCommand{

    IntakeSubsystem intakeSubsystem;

    public AutoStopIntakeCommand(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        intakeSubsystem.stopMotor();
    }
}
