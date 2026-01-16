package frc.robot.utils;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Robot;

public class Helpers {
	public static boolean isBabycakes() {
		return RobotController.getComments().toUpperCase().equals("BABYCAKES");
	}

	public static boolean isBlackout() {
		return RobotController.getComments().toUpperCase().equals("BLACKOUT");
	}

	public static String getRobotName() {
		if (Robot.isSimulation()) {
			return "(Simulation)";
		} else if (isBabycakes() || isBlackout()) {
			return RobotController.getComments();
		} else {
			return RobotController.getComments() + " (Compbot)";
		}
	}
}
