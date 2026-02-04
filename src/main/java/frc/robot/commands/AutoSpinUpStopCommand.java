package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoSpinUpStopCommand extends InstantCommand{

    ShooterSubsystem shooterSubsystem;

    public AutoSpinUpStopCommand(ShooterSubsystem shooterSubsystem){
        addRequirements(shooterSubsystem);
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
	public void initialize() {
        shooterSubsystem.stop();
	}

}