package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class RobotCentricDriveCommand extends Command {
    private DriveSubsystem driveSubsystem;
    private double xSpeed;
    private double ySpeed;

    public RobotCentricDriveCommand(DriveSubsystem driveSubsystem, double xSpeed, double ySpeed) {
        this.driveSubsystem = driveSubsystem;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        driveSubsystem.drive(this.xSpeed, this.ySpeed, 0, false);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}