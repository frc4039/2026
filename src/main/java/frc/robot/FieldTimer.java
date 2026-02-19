package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Ellipse2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public class FieldTimer {
     
    public static Alliance getAlliance() {
        if(DriverStation.getAlliance().isPresent()) {
            if(DriverStation.getAlliance().equals(Alliance.Red)) {
                return Alliance.Red;
            } else {
                return Alliance.Blue;
            }
        } else{
            return Alliance.Blue;
        }
    }
    
    public static double getMatchTime() {
        if (DriverStation.isAutonomous()) {
            if (DriverStation.getMatchTime() < 0) return DriverStation.getMatchTime();
            return 20 - DriverStation.getMatchTime();
        } else if (DriverStation.isTeleop()) {
            if (DriverStation.getMatchTime() < 0) return DriverStation.getMatchTime();
            return 160 - DriverStation.getMatchTime();
        }
        return -1;
    }
 
    
    public static Optional<Shift> getCurrentShift() {
        double matchTime = getMatchTime();
        if (matchTime < 0) return Optional.empty();

        for (Shift shift : Shift.values()) {
            if (matchTime < shift.endTime) {
                return Optional.of(shift);
            }
        }
        return Optional.empty();
    }



    public static Optional<Time> timeRemainingInCurrentShift() {
        return getCurrentShift().map((shift) -> Seconds.of(shift.endTime - getMatchTime()));
    }

    public static boolean isActive(Alliance alliance, Shift shift) {
        Optional<Alliance> autoWinner = getAutoWinner();
        switch (shift.activeType) {
            case BOTH:
                return true;
            case AUTO_WINNER:
                return autoWinner.isPresent() && autoWinner.get() == alliance;
            case AUTO_LOSER:
                return autoWinner.isPresent() && autoWinner.get() != alliance;
            default:
                return false;
        }
    }
    
    public static boolean isActive(Alliance alliance) {
        Optional<Shift> currentShift = getCurrentShift();
        return currentShift.isPresent() && isActive(alliance, currentShift.get());
    }

     public static Optional<Alliance> getAutoWinner() {
        String msg = DriverStation.getGameSpecificMessage();
        char msgChar = msg.length() > 0 ? msg.charAt(0) : ' ';
        switch (msgChar) {
            case 'B':
                return Optional.of(Alliance.Blue);
            case 'R':
                return Optional.of(Alliance.Red);
            default:
                return Optional.of(Alliance.Red);
        }
    }

    public enum Shift {
        AUTO(0, 20, ActiveType.BOTH),
        TRANSITION(20, 30, ActiveType.BOTH),
        SHIFT_1(30, 55, ActiveType.AUTO_LOSER),
        SHIFT_2(55, 80, ActiveType.AUTO_WINNER),
        SHIFT_3(80, 105, ActiveType.AUTO_LOSER),
        SHIFT_4(105, 130, ActiveType.AUTO_WINNER),
        ENDGAME(130, 160, ActiveType.BOTH);

        final int startTime;
        final int endTime;
        final ActiveType activeType;

        private Shift(int startTime, int endTime, ActiveType activeType) {
            this.startTime = startTime;
            this.endTime = endTime;
            this.activeType = activeType;
        }
    }

    private enum ActiveType {
        BOTH,
        AUTO_WINNER,
        AUTO_LOSER
    }

    public double getSecondsRemaining() {
    Optional<Double> secondsRemaining =timeRemainingInCurrentShift().map(time -> time.in(Seconds));	
        if(secondsRemaining.isPresent()) {
            return secondsRemaining.get();
        } else {
            return -1.0;
        }
    }    
    public void initSendable(SendableBuilder builder) {
		builder.addBooleanProperty("Active Shift", () -> isActive(getAlliance()), null);
		builder.addDoubleProperty("Time Left In Shift", () -> getSecondsRemaining(), null);
	}
}
