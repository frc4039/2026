// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.RobotCentricDriveCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.utils.HardwareMonitor;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	private HardwareMonitor hardwareMonitor = new HardwareMonitor();

	// The robot's subsystems and commands are defined here...
	private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
	private final DriveSubsystem driveSubsystem = new DriveSubsystem(hardwareMonitor);

	// Replace with CommandPS4Controller or CommandJoystick if needed
	private final CommandXboxController driver = new CommandXboxController(
			OperatorConstants.kDriverControllerPort);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		driveSubsystem.setDefaultCommand(
				new TeleopDriveCommand(driveSubsystem, driver::getLeftY, driver::getLeftX,
						driver::getRightX, -1.0));

		// Configure the trigger bindings
		configureBindings();

		hardwareMonitor.registerDevice(null, driver);
		SmartDashboard.putData("Hardware Errors", hardwareMonitor);
		SmartDashboard.putData(CommandScheduler.getInstance());

		SmartDashboard.putData(driveSubsystem);
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
	 * constructor with an arbitrary predicate, or via the named factories in
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
	 * for {@link CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
	 * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
	 * Flight joysticks}.
	 */
	private void configureBindings() {
		driver.start().onTrue(new InstantCommand(() -> {
			Rotation2d resetAngle = Rotation2d.fromDegrees(0);
			Optional<Alliance> alliance = DriverStation.getAlliance();
			if (alliance.isPresent() && alliance.get() == Alliance.Red) {
				resetAngle = Rotation2d.fromDegrees(180);
			}
			Translation2d currentPosition = driveSubsystem.getPose().getTranslation();
			driveSubsystem.resetGyroAngle(new Pose2d(currentPosition, resetAngle));
		}));
		// Schedule `exampleMethodCommand` when the Xbox controller's B button is
		// pressed, cancelling on release.
		driver.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

		// Robot centric driving
		driver.povUp().whileTrue(new RobotCentricDriveCommand(driveSubsystem, 0.035, 0));
		driver.povDown().whileTrue(new RobotCentricDriveCommand(driveSubsystem, -0.035, 0));
		driver.povLeft().whileTrue(new RobotCentricDriveCommand(driveSubsystem, 0, 0.035));
		driver.povRight().whileTrue(new RobotCentricDriveCommand(driveSubsystem, 0, -0.035));

	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	// public Command getAutonomousCommand() {
	// // An example command will be run in autonomous
	// return Autos.exampleAuto(m_exampleSubsystem);
	// }
}
