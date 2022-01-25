package frc.subsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.utility.ControllerDriveInputs;
import frc.utility.Limelight;
import frc.utility.MathUtil;
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
        ChassisSpeeds currentRobotState = drive.getRobotState();
        Translation2d currentRobotVelocity = new Translation2d(currentRobotState.vxMetersPerSecond,
                currentRobotState.vyMetersPerSecond);

        Translation2d predictedPosition = predictFutureTranslation(0.2, getCurrentTranslation(), currentRobotVelocity);
        double neededBallVelocity = getNeededBallVelocity(predictedPosition.getNorm());

        Translation2d neededEjectionVector = new Translation2d(neededBallVelocity,
                Math.atan2(predictedPosition.getY(), predictedPosition.getX()));

        Translation2d velocityCompensatedEjectionVector = getVelocityCompensatedEjectionVector(currentRobotVelocity,
                neededEjectionVector);

        double wantedAngle = Math.atan2(velocityCompensatedEjectionVector.getY(), velocityCompensatedEjectionVector.getX());

        drive.updateTurn(controllerDriveInputs.getX(), controllerDriveInputs.getY(), new Rotation2d(wantedAngle));

        //TODO: set the shooter speed
    }

    // Calculate Current Robot Position
    public Translation2d getCurrentTranslation() {
        Rotation2d currentGyroAngle = RobotTracker.getInstance().getGyroAngle();

        double distanceToTarget = limelight.getDistance() + Constants.GOAL_RADIUS;
        double angleToTarget = 180 + currentGyroAngle.getDegrees() - limelight.getHorizontalOffset();
        MutableTranslation2d currentPose = new MutableTranslation2d(distanceToTarget * Math.cos(Math.toRadians(angleToTarget)),
                distanceToTarget * Math.sin(Math.toRadians(angleToTarget))).minus(Constants.LIMELIGHT_CENTER_OFFSET).plus(
                Constants.GOAL_POSITION);
        return currentPose.getTranslation2d();
    }

    // Predict Position In The Future

    public Translation2d predictFutureTranslation(double predictAheadTime, Translation2d currentTranslation,
                                                  Translation2d currentVelocity) {
        return currentTranslation.plus(currentVelocity.times(predictAheadTime));
    }

    // TODO: actually calculate this
    public double getNeededBallVelocity(double distance) {
        return distance * 5;
    }

    @SuppressWarnings("NewMethodNamingConvention")
    public Translation2d getVelocityCompensatedEjectionVector(Translation2d robotVelocityVector,
                                                              Translation2d wantedEjectionVector) {
        return wantedEjectionVector.minus(robotVelocityVector);
    }

    /**
     * Adds a vision update to the robot tracker even if the calculated pose is too far from the expected pose.
     */
    public void forceUpdatePose() {

    }

    @Override
    public void update() {
        Translation2d currentTranslation = getCurrentTranslation();
        if (MathUtil.dist2(robotTracker.getPoseMeters().getTranslation(),
                currentTranslation) < Constants.VISION_MANAGER_DISTANCE_THRESHOLD) {
            logData("Using Vision Info", Boolean.TRUE);
        } else {
            logData("Using Vision Info", Boolean.FALSE);
        }
    }


    @Override
    public void close() throws Exception {

    }
}
