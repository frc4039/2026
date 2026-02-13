// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualVelocityCommand;
import frc.robot.commands.MoveIntakeSlideCommand;
import frc.robot.commands.OwlHeadTurretCommand;
import frc.robot.commands.ResetTurretGyro;
import frc.robot.commands.AimCommand;
import frc.robot.commands.AlignToTowerCommand;
import frc.robot.commands.AlignToTowerCommandGroup;
import frc.robot.commands.Autos;
import frc.robot.commands.SpindexerCommand;
import frc.robot.commands.TurretMoveCommand;
import frc.robot.commands.TurretWithJoystickCommand;
import frc.robot.commands.ZeroIntakeSlideCommand;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterHoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSlideSubsystem;
import frc.robot.commands.RobotCentricDriveCommand;
import frc.robot.commands.ShooterHoodCommand;
import frc.robot.commands.SpinUpCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.TurretAprilTagAimCommand;
import frc.robot.commands.FeederCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.utils.HardwareMonitor;
import frc.robot.utils.Helpers;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
  // The robot's subsystems and commands are defined here...
 
	private HardwareMonitor hardwareMonitor = new HardwareMonitor();
 	private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMonitor);
	private final DriveSubsystem driveSubsystem = new DriveSubsystem(hardwareMonitor);
	private final TurretSubsystem turretSubsystem = new TurretSubsystem(driveSubsystem, hardwareMonitor);
	private final VisionSubsystem visionSubsystem = new VisionSubsystem(driveSubsystem, turretSubsystem);
	private final FeederSubsystem turretFeederSubsystem = new FeederSubsystem(hardwareMonitor);
	private final ShooterHoodSubsystem shooterHoodSubsystem = new ShooterHoodSubsystem();
	private final SpindexerSubsystem spindexerSubsystem = new SpindexerSubsystem(hardwareMonitor);
	private final IntakeSlideSubsystem intakeSlideSubsystem = new IntakeSlideSubsystem();

	private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

	private final CommandXboxController driver = new CommandXboxController(
			OperatorConstants.kDriverControllerPort);

  private final CommandXboxController operator = 
    new CommandXboxController(OperatorConstants.kOperatorPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    
    SmartDashboard.putData(
      new InstantCommand(() -> turretSubsystem.resetTurret())
        .withName("ResetTurret")
        .ignoringDisable(true)
    );
	SmartDashboard.putData(spindexerSubsystem);
	SmartDashboard.putData(intakeSubsystem);
	SmartDashboard.putData(turretSubsystem);
	SmartDashboard.putData(shooterSubsystem);
	SmartDashboard.putData(shooterHoodSubsystem);
	SmartDashboard.putData(intakeSlideSubsystem);


	hardwareMonitor.registerDevice(null, driver);
		SmartDashboard.putData("Hardware Errors", hardwareMonitor);
		SmartDashboard.putData(CommandScheduler.getInstance());

		SmartDashboard.putData(driveSubsystem);

		 // Put the BuildInfo so we can see what version of the code is running.
    SmartDashboard.putData("BuildInfo", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.publishConstString("Robot Name", Helpers.getRobotName());
        builder.publishConstString("Git Branch", Helpers.getGitBranch());
        builder.publishConstString("Git SHA", Helpers.getGitSHA());
        builder.publishConstString("Build Date", Helpers.getBuildDate());
      }
    });
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

		driveSubsystem.setDefaultCommand(
				new TeleopDriveCommand(driveSubsystem, driver::getLeftY, driver::getLeftX,
						driver::getRightX, -1.0));

		// Robot centric driving
		driver.povUp().whileTrue(new RobotCentricDriveCommand(driveSubsystem, 0.035, 0));
		driver.povDown().whileTrue(new RobotCentricDriveCommand(driveSubsystem, -0.035, 0));
		driver.povLeft().whileTrue(new RobotCentricDriveCommand(driveSubsystem, 0, 0.035));
		driver.povRight().whileTrue(new RobotCentricDriveCommand(driveSubsystem, 0, -0.035));


    	driver.leftTrigger().whileTrue(new IntakeCommand(intakeSubsystem, true));
    	driver.leftBumper().whileTrue(new IntakeCommand(intakeSubsystem, false));

		driver.rightBumper().whileTrue(new SpindexerCommand(spindexerSubsystem, true).alongWith(new FeederCommand(turretFeederSubsystem, true)));
		driver.rightTrigger().whileTrue(new SpinUpCommand(shooterSubsystem, turretSubsystem));
      	driver.x().whileTrue(new MoveIntakeSlideCommand(intakeSlideSubsystem, true));
		driver.b().onTrue(new ResetTurretGyro(turretSubsystem).ignoringDisable(true));
		driver.a().onTrue(new AimCommand(turretSubsystem, driveSubsystem, shooterHoodSubsystem));
		driver.y().whileTrue(new MoveIntakeSlideCommand(intakeSlideSubsystem, false));
	    //driver.rightTrigger().whileTrue(new OwlHeadTurretCommand(() -> driveSubsystem.getHeading(), turretSubsystem));
	    
		//driver.rightBumper().whileTrue(new AlignToTowerCommandGroup(driveSubsystem, visionSubsystem));
		//driver.x().whileTrue(new ShooterHoodCommand(shooterHoodSubsystem, 5));
		// driver.rightBumper().whileTrue(new AlignToTowerCommandGroup(driveSubsystem, visionSubsystem));
		// driver.x().whileTrue(new ShooterHoodCommand(shooterHoodSubsystem, 5));
		// operator.axisMagnitudeGreaterThan(XboxController.Axis.kRightX.value, 0.25)
		// 	.or(
		// 		operator.axisMagnitudeGreaterThan(XboxController.Axis.kRightY.value, 0.25)
		// 	)
		// 	.whileTrue(
		// 		new TurretWithJoystickCommand(
		// 			turretSubsystem,
		// 			() -> operator.getRightX(),
		// 			() -> operator.getRightY(),
		// 			() -> driveSubsystem.getHeading()
		// 		)
		// 	); //moves turrettttttttttttttttttttt

		// operator.leftTrigger().whileTrue(new SpindexerCommand(spindexerSubsystem, true));
		// operator.leftBumper().whileTrue(new SpindexerCommand(spindexerSubsystem, false));

		// operator.rightTrigger().whileTrue(new IntakeCommand(intakeSubsystem, true));
		// operator.rightBumper().whileTrue(new FeederCommand(turretFeederSubsystem, false));

		// operator.a().onTrue(new ShooterHoodCommand(shooterHoodSubsystem, 70));
		// operator.b().onTrue(new InstantCommand(() -> shooterHoodSubsystem.resetTurret()).ignoringDisable(true));

		operator.a().whileTrue(new MoveIntakeSlideCommand(intakeSlideSubsystem, true));
		operator.b().whileTrue(new MoveIntakeSlideCommand(intakeSlideSubsystem, false));
		operator.y().onTrue(new ZeroIntakeSlideCommand(intakeSlideSubsystem).ignoringDisable(true));


	}

}
