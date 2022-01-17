package frc.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.utility.ControllerDriveInputs;
import frc.utility.Limelight;
import org.jetbrains.annotations.Nullable;

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
        @Nullable ChassisSpeeds currentRobotState = drive.getRobotState();
        if (currentRobotState == null) return;
        Translation2d currentRobotVelocity = new Translation2d(currentRobotState.vxMetersPerSecond,
                currentRobotState.vyMetersPerSecond);

        Translation2d predictedPosition = predictFutureTranslation(0.2, getCurrentPose().getTranslation(), currentRobotVelocity);
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
    public Pose2d getCurrentPose() {
        Rotation2d currentGyroAngle = RobotTracker.getInstance().getGyroAngle();
        double limelightHorizontalOffset = -limelight.getHorizontalOffset();
        double distanceToTarget = limelight.getDistance();
        double angleToTarget = 180 + currentGyroAngle.getDegrees() - limelightHorizontalOffset;
        Translation2d currentPose = new Translation2d(distanceToTarget * Math.cos(Math.toRadians(angleToTarget)),
                distanceToTarget * Math.sin(Math.toRadians(angleToTarget)));
        return new Pose2d(currentPose, currentGyroAngle);
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


    @Override
    public void close() throws Exception {

    }


}
