// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AlignToTowerCommandGroup extends SequentialCommandGroup {
	public AlignToTowerCommandGroup(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
		addCommands(new AlignToTowerCommand(driveSubsystem, visionSubsystem),
				new FinalTowerAlignCommand(driveSubsystem, visionSubsystem));
	}
}
