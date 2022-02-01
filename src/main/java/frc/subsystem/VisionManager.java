package frc.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.utility.ControllerDriveInputs;
import frc.utility.Limelight;
import frc.utility.MathUtil;
import frc.utility.Timer;
import frc.utility.geometry.MutableTranslation2d;

public class VisionManager extends AbstractSubsystem {
    private static VisionManager instance = new VisionManager();

    RobotTracker robotTracker = RobotTracker.getInstance();
    Limelight limelight = Limelight.getInstance();
    Drive drive = Drive.getInstance();

    private VisionManager() {
        super(Constants.VISION_MANAGER_PERIOD);
    }

    public static VisionManager getInstance() {
        return instance;
    }

    @Override
    public void selfTest() {

    }

    @Override
    public void logData() {

    }

    public void shootAndMove(ControllerDriveInputs controllerDriveInputs) {
    }

    /**
     * @return the current translation of the robot based on the vision data
     */
    private MutableTranslation2d getCurrentTranslation() {
        Rotation2d currentGyroAngle = RobotTracker.getInstance().getGyroAngle();

        double distanceToTarget = limelight.getDistance() + Constants.GOAL_RADIUS;
        double angleToTarget = 180 + currentGyroAngle.getDegrees() - limelight.getHorizontalOffset();
        return new MutableTranslation2d(distanceToTarget * Math.cos(Math.toRadians(angleToTarget)),
                distanceToTarget * Math.sin(Math.toRadians(angleToTarget)))
                .minus(Constants.LIMELIGHT_CENTER_OFFSET).plus(Constants.GOAL_POSITION);
    }


    /*
     * @return the current position of the robot based on a translation and some time. It adds the current velocity * time to
     * the translation.
     */
    private MutableTranslation2d predictFutureTranslation(double predictAheadTime, MutableTranslation2d currentTranslation,
                                                          Translation2d currentVelocity) {
        return currentTranslation.plus(currentVelocity.times(predictAheadTime));
    }


    /**
     * @param dist  distance to target (meters)
     * @param angle angle of the hood (radians)
     * @return horizontal velocity needed to hit target
     */
    private double getNeededHorizontalBallVelocity(double dist, double angle) {
        //@formatter:off
        return Math.sqrt((0.5 * Constants.GRAVITY * dist) / (Math.tan(angle) - ((Constants.GOAL_HEIGHT - Constants.SHOOTER_HEIGHT) / dist)));
        //@formatter:on
    }

    /**
     * @param speed     wanted ejection velocity
     * @param hoodAngle angle of the hood
     */
    private void getShooterRPM(double speed, double hoodAngle) {

    }


    @SuppressWarnings("NewMethodNamingConvention")
    private MutableTranslation2d getVelocityCompensatedEjectionVector(Translation2d robotVelocityVector,
                                                                      MutableTranslation2d wantedEjectionVector) {
        return wantedEjectionVector.minus(robotVelocityVector);
    }

    /**
     * Adds a vision update to the robot tracker even if the calculated pose is too far from the expected pose.
     */
    private void forceUpdatePose() {
        robotTracker.resetPosition(
                new Pose2d(getCurrentTranslation().getTranslation2d(), robotTracker.getLatencyCompedPoseMeters().getRotation()));
    }

    public void autoTurnRobotToTarget(ControllerDriveInputs controllerDriveInputs, boolean fieldRelative) {
        double degreeOffset = Limelight.getInstance().getHorizontalOffset();
        drive.updateTurn(controllerDriveInputs.getX(), controllerDriveInputs.getY(),
                robotTracker.getGyroAngle().rotateBy(Rotation2d.fromDegrees(-degreeOffset)), fieldRelative);
    }


    /**
     * Calculates and sets the flywheel speed considering a static robot velocity
     */
    public void updateFlywheelSpeedForStaticPose() {

    }

    /**
     * Calculates and sets the flywheel speed considering a moving robot
     */
    public void updateFlywheelSpeed() {

    }

    @Override
    public void update() {
        MutableTranslation2d robotTranslation = getCurrentTranslation();
        if (MathUtil.dist2(robotTracker.getLatencyCompedPoseMeters().getTranslation(),
                robotTranslation) < Constants.VISION_MANAGER_DISTANCE_THRESHOLD_SQUARED) {
            robotTracker.addVisionMeasurement(
                    new Pose2d(robotTranslation.getTranslation2d(), robotTracker.getLatencyCompedPoseMeters().getRotation()),
                    Timer.getFPGATimestamp() - limelight.getLatency() - 0.011);
            logData("Using Vision Info", Boolean.TRUE);
        } else {
            logData("Using Vision Info", Boolean.FALSE);
        }
    }


    @Override
    public void close() throws Exception {

    }
}
