package frc.robot.utils;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.TurretSubsystem.TurretConstants;

public class ShotCalculator {
    private final InterpolatingDoubleTreeMap shootingEstimator = new InterpolatingDoubleTreeMap();
    private final Mechanism2d previewMechanism = new Mechanism2d(8, 5);
    private final MechanismRoot2d previewStart = previewMechanism.getRoot("start", 0,
            TurretConstants.kTurretOffset.getZ());
    private final MechanismRoot2d previewTarget = previewMechanism.getRoot("target", 0, 0);
    private final static double previewTime = TurretConstants.kTimeOfFlight;
    private final MechanismLigament2d previewTrajectory[] = new MechanismLigament2d[10];

    public ShotCalculator() {
        shootingEstimator.put(7.517, 27.0);
        shootingEstimator.put(7.77, 31.0);
        shootingEstimator.put(8.24, 35.5);
        shootingEstimator.put(7.39, 24.0);

        // Draw a crosshairs for the target.
        double crossLength = 0.1;
        double crossWidth = 2;
        Color8Bit crossColour = new Color8Bit(Color.kRed);
        previewTarget.append(new MechanismLigament2d("right", crossLength, 0, crossWidth, crossColour));
        previewTarget.append(new MechanismLigament2d("up", crossLength, 90, crossWidth, crossColour));
        previewTarget.append(new MechanismLigament2d("left", crossLength, 180, crossWidth, crossColour));
        previewTarget.append(new MechanismLigament2d("down", crossLength, 270, crossWidth, crossColour));

        // Draw an arc of the shot.
        Color8Bit trajectoryColour = new Color8Bit(Color.kBlue);
        MechanismObject2d previousSegment = previewStart;
        for (int i = 0; i < previewTrajectory.length; i++) {
            previewTrajectory[i] = new MechanismLigament2d(Integer.toString(i), 0, 0, 2, trajectoryColour);
            previousSegment.append(previewTrajectory[i]);
            previousSegment = previewTrajectory[i];
        }

        SmartDashboard.putData("ShotSimulation", previewMechanism);
    }

    // Desired target location in 3d space.
    private Translation3d target = FieldConstants.kBlueHub;

    // Robot-relative turret angle
    public double turretYaw;
    // Hood pitch
    public double hoodPitch;

    private double shooterMps;
    // Rotations per second for shooter
    public double shooterRps;

    private ChassisSpeeds robotSpeeds;
    private Pose3d robotPosition;

    public void setTarget(Translation3d target) {
        if (!this.target.equals(target)) {
            this.target = target;
            this.calculate();
        }
    }

    public void update(Pose3d robotPosition, ChassisSpeeds robotSpeeds) {
        this.robotPosition = robotPosition;
        this.robotSpeeds = robotSpeeds;
        this.calculate();
    }

    @SuppressWarnings("unused")
    public void calculate() {
        Translation3d robotVelocity = new Translation3d(
                robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond, 0);

        Pose3d ballReleasePosition = robotPosition.plus(TurretConstants.kTurretOffset);

            Pose2d currentRobotPose2d = robotPosition.plus(TurretConstants.kTurretOffset).toPose2d();
            Pose2d hubPose2d = new Pose2d(this.target.toTranslation2d(), new Rotation2d());
            double distanceToHub = currentRobotPose2d.getTranslation().getDistance(hubPose2d.getTranslation());
            
            // From TurretSubsystem.getXVelocity()
            double xVelocity = distanceToHub / TurretConstants.kTimeOfFlight;

            // From TurretSubsystem.getOutputVelocity()
		    this.shooterMps = Math.sqrt(Math.pow(xVelocity, 2) + Math.pow(TurretConstants.kVelocityZ, 2));
            this.turretYaw = -1 * hubPose2d.relativeTo(currentRobotPose2d).getTranslation().getAngle().getDegrees();
            
            // From TurretSubsystem.getHoodAngle()
            this.hoodPitch = Units.radiansToDegrees(Math.atan2(TurretConstants.kVelocityZ, xVelocity));
    

        // TODO: Calculate the correct shot.
        Translation3d vecT = new Translation3d(
            (Math.cos(Units.degreesToRadians(this.turretYaw))) * (Math.cos(Units.degreesToRadians(this.hoodPitch))) * (this.shooterMps), 
            (Math.sin(Units.degreesToRadians(this.turretYaw))) * (Math.cos(Units.degreesToRadians(this.hoodPitch))) * (this.shooterMps), 
            (Math.sin(this.hoodPitch)) * this.shooterMps);
        
        Translation3d ballReleaseVelocity = new Translation3d(
            (vecT.getX()) - (robotVelocity.getX()), 
            (vecT.getY()) - (robotVelocity.getY()), 
            vecT.getZ());

            ballReleaseVelocity = null;
        if (ballReleaseVelocity != null) {
            // Convert the theoretical shot into the real values.
            this.shooterMps = ballReleaseVelocity.getNorm();

            double ballDirX = ballReleaseVelocity.getX();
            double ballDirY = ballReleaseVelocity.getY();
            this.turretYaw = new Rotation2d(ballReleaseVelocity.getX(), ballReleaseVelocity.getY())
                    .minus(ballReleasePosition.getRotation().toRotation2d().plus(new Rotation2d(Units.degreesToRadians(180))))
                    .getDegrees();

            double ballDirXY = Math.sqrt(ballDirX * ballDirX + ballDirY * ballDirY);
            double ballDirZ = ballReleaseVelocity.getZ();
            this.hoodPitch = new Rotation2d(ballDirXY, ballDirZ).getDegrees();
        }

        // Convert ball m/s to rotations/s.
        this.shooterRps = shootingEstimator.get(shooterMps);

        this.simulateShot();
    }

    private void simulateShot() {
        Pose3d ballReleasePosition = robotPosition.plus(TurretConstants.kTurretOffset);
        Vector<N3> position = ballReleasePosition.getTranslation().toVector();
        Vector<N3> velocity = new Translation3d(
                robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond, 0).toVector();
        velocity = velocity.plus(
                new Translation3d(this.shooterMps, 0.0, 0.0)
                        .rotateBy(new Rotation3d(
                                0.0,
                                Units.degreesToRadians(-this.hoodPitch),
                                Units.degreesToRadians(-this.turretYaw
                                        + ballReleasePosition.getRotation().toRotation2d().getDegrees())))
                        .toVector());
        Vector<N3> acceleration = new Translation3d(0, 0, -9.81).toVector();
        double timeStep = previewTime / (double) previewTrajectory.length;

        // Update trajectory preview by simulating shot.
        Translation3d startToTarget = this.target.minus(ballReleasePosition.getTranslation());
        double startDistance = startToTarget.getNorm();
        double lastXY = 0;
        double lastZ = ballReleasePosition.getZ();
        double lastAngle = 0;
        for (int i = 0; i < previewTrajectory.length; i++) {
            velocity = velocity.plus(acceleration.times(timeStep / 2.0));
            position = position.plus(velocity.times(timeStep));
            velocity = velocity.plus(acceleration.times(timeStep / 2.0));

            double xy = new Translation3d(position).toTranslation2d().minus(ballReleasePosition.getTranslation().toTranslation2d()).getNorm();
            double z = position.get(2);
            Translation2d delta = new Translation2d(xy - lastXY, z - lastZ);
            previewTrajectory[i].setLength(delta.getNorm());
            double angle = delta.getAngle().getDegrees();
            previewTrajectory[i].setAngle(angle - lastAngle);

            lastXY = xy;
            lastZ = z;
            lastAngle = angle;
        }

        // Update target crosshairs.
        previewTarget.setPosition(
                startDistance,
                this.target.getZ());
    }

    public Pose2d getTargetPose() {
        return new Pose2d(this.target.toTranslation2d(), new Rotation2d());
    }
}
