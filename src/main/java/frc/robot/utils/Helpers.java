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

	 public static String getGitBranch() {
        String dirtyString = " (dirty)";
        //TODO: add this back when we know where Build Constants was moved to -> also make it return a String again
        switch (BuildConstants.DIRTY) {
             case 0:
                dirtyString = "";
        }
        return BuildConstants.GIT_BRANCH + dirtyString;
    }
    
    public static String getGitSHA() {
        return BuildConstants.GIT_SHA;
    }
    
    public static String getBuildDate() {
        return BuildConstants.BUILD_DATE;
    }
}
