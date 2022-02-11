package frc.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.utility.ControllerDriveInputs;
import frc.utility.Limelight;
import frc.utility.Limelight.CamMode;
import frc.utility.Limelight.LedMode;
import frc.utility.MathUtil;
import frc.utility.Timer;
import frc.utility.geometry.MutableTranslation2d;
import frc.utility.shooter.visionlookup.ShooterConfig;
import frc.utility.shooter.visionlookup.ShooterPreset;
import frc.utility.shooter.visionlookup.VisionLookUpTable;
import org.jetbrains.annotations.Contract;

import java.util.ArrayList;

import static frc.utility.OrangeUtility.getSpeed;
import static frc.utility.OrangeUtility.getTranslation2d;

public final class VisionManager extends AbstractSubsystem {
    private static VisionManager instance = new VisionManager();

    RobotTracker robotTracker = RobotTracker.getInstance();
    Limelight limelight = Limelight.getInstance();
    Drive drive = Drive.getInstance();
    Shooter shooter = Shooter.getInstance();

    private final VisionLookUpTable visionLookUpTable = VisionLookUpTable.getInstance();
    private BallPredictionMode ballPredictionMode = BallPredictionMode.VELOCITY_COMPENSATED;
    private double shooterHoodAngleBias = 0;

    {
        logData("Shooter Angle Bias", shooterHoodAngleBias);
    }

    public void adjustShooterHoodBias(double amount) {
        shooterHoodAngleBias += amount;
        logData("Shooter Angle Bias", shooterHoodAngleBias);
    }

    public double getShooterHoodAngleBias() {
        return shooterHoodAngleBias;
    }

    public void setShooterConfig(ShooterConfig shooterConfig) {
        visionLookUpTable.setShooterConfig(shooterConfig);
    }

    public enum BallPredictionMode {
        VELOCITY_COMPENSATED, STATIC_POSE
    }

    public BallPredictionMode getBallPredictionMode() {
        return ballPredictionMode;
    }


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

    public void shootAndMove(ControllerDriveInputs controllerDriveInputs, boolean useFieldRelative) {
        if (ballPredictionMode == BallPredictionMode.STATIC_POSE) {
            // STATIC_POSE prediction mode doesn't use the robot tracker so if we're using it, we want to use the turn method
            // that uses pure vision data to calculate the turn
            autoTurnRobotToTarget(controllerDriveInputs, useFieldRelative);
            updateShooterStateStaticPose(); // TODO: this call will be doubled if the operator is also trying to turn the flywheel on.
            return;
        }

        updateShooterState(); // TODO: this call will be doubled if the operator is also trying to turn the flywheel on.
        MutableTranslation2d relativeRobotPosition = predictTranslationAtZeroVelocity(
                robotTracker.getLatencyCompedChassisSpeeds(),
                robotTracker.getLatencyCompedPoseMeters().getTranslation()).minus(Constants.GOAL_POSITION);
        Rotation2d targetRotation = new Rotation2d(Math.atan2(relativeRobotPosition.getY(), relativeRobotPosition.getX()));
        drive.updateTurn(controllerDriveInputs, targetRotation, useFieldRelative);

        shooter.setFiring(!drive.isAiming());
    }

    /**
     * @return the current translation of the robot based on the vision data
     */
    @Contract(pure = true)
    private MutableTranslation2d getCurrentTranslation() {
        Rotation2d currentGyroAngle = getLatencyCompedLimelightRotation();

        double distanceToTarget = limelight.getDistance() + Constants.GOAL_RADIUS;
        double angleToTarget = 180 + currentGyroAngle.getDegrees() - limelight.getHorizontalOffset();
        return new MutableTranslation2d(distanceToTarget * Math.cos(Math.toRadians(angleToTarget)),
                distanceToTarget * Math.sin(Math.toRadians(angleToTarget)))
                .minus(Constants.LIMELIGHT_CENTER_OFFSET.rotateBy(currentGyroAngle)).plus(Constants.GOAL_POSITION);
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
     * @return the position of the robot when it hits {@link Constants#MAX_SHOOT_SPEED} if the robot starts decelerating
     * immediately.
     */
    @Contract(pure = true)
    private MutableTranslation2d predictTranslationAtZeroVelocity(ChassisSpeeds currentSpeeds,
                                                                  Translation2d currentTranslation) {
        MutableTranslation2d predictedTranslation;
        double speed = getSpeed(robotTracker.getLatencyCompedChassisSpeeds());
        if (speed > Constants.MAX_SHOOT_SPEED) {
            double time = (speed - Constants.MAX_SHOOT_SPEED) / Constants.MAX_ACCELERATION;
            MutableTranslation2d velocity = getTranslation2d(currentSpeeds);
            predictedTranslation = velocity.times(((speed + Constants.MAX_SHOOT_SPEED) / 2) * time).plus(currentTranslation);
        } else {
            predictedTranslation = new MutableTranslation2d(currentTranslation);
        }

        logData("Predicted Future Pose X", predictedTranslation.getX());
        logData("Predicted Future Pose Y", predictedTranslation.getY());
        logData("Predicted Future Pose Time", Timer.getFPGATimestamp());

        return predictedTranslation;
    }


    /**
     * @param dist  distance to target (meters)
     * @param angle angle of the hood (radians)
     * @return horizontal velocity needed to hit target
     */
    @Contract(pure = true)
    private double getNeededHorizontalBallVelocity(double dist, double angle) {
        //@formatter:off
        return Math.sqrt((0.5 * Constants.GRAVITY * dist) / (Math.tan(angle) - ((Constants.GOAL_HEIGHT - Constants.SHOOTER_HEIGHT) / dist)));
        //@formatter:on
    }

    /**
     * @param speed     wanted ejection velocity
     * @param hoodAngle angle of the hood
     */
    @Contract(pure = true)
    private void getShooterRPM(double speed, double hoodAngle) {

    }


    @SuppressWarnings("NewMethodNamingConvention")
    @Contract(pure = true)
    private MutableTranslation2d getVelocityCompensatedEjectionVector(Translation2d robotVelocityVector,
                                                                      MutableTranslation2d wantedEjectionVector) {
        return wantedEjectionVector.minus(robotVelocityVector);
    }

    /**
     * Adds a vision update to the robot tracker even if the calculated pose is too far from the expected pose.
     * <p>
     * You need to call {@link #forceVisionOn(Object)} before calling this method.
     */
    public void forceUpdatePose() {
        limelight.setCamMode(CamMode.VISION_PROCESSOR);
        limelight.setLedMode(LedMode.ON);
        if (limelight.isTargetVisible()) {
            robotTracker.addVisionMeasurement(new Pose2d(getCurrentTranslation().getTranslation2d(),
                    getLatencyCompedLimelightRotation()), getLimelightTime());
        }
    }

    public void autoTurnRobotToTarget(ControllerDriveInputs controllerDriveInputs, boolean fieldRelative) {
        double degreeOffset;
        if (limelight.isTargetVisible()) {
            degreeOffset = Limelight.getInstance().getHorizontalOffset();
            Rotation2d targetRotation = getLatencyCompedLimelightRotation().rotateBy(Rotation2d.fromDegrees(-degreeOffset));
            drive.updateTurn(controllerDriveInputs, targetRotation, fieldRelative);
        } else {
            //Use best guess if no target is visible
            Translation2d relativeRobotPosition = robotTracker.getLatencyCompedPoseMeters().getTranslation()
                    .minus(Constants.GOAL_POSITION);
            Rotation2d targetRotation = new Rotation2d(Math.atan2(relativeRobotPosition.getY(), relativeRobotPosition.getX()));
            drive.updateTurn(controllerDriveInputs, targetRotation, fieldRelative);
        }

        shooter.setFiring(limelight.isTargetVisible() && !drive.isAiming());
    }

    public Rotation2d getLatencyCompedLimelightRotation() {
        return robotTracker.getGyroRotation(getLimelightTime());
    }

    /**
     * @return the time of the last vision update in seconds
     */
    private double getLimelightTime() {
        return Timer.getFPGATimestamp() - limelight.getLatency() - 0.011;
    }


    /**
     * Calculates and sets the flywheel speed considering a static robot velocity
     */
    public void updateShooterStateStaticPose() {
        ballPredictionMode = BallPredictionMode.STATIC_POSE;

        double distanceToTarget;
        if (limelight.isTargetVisible()) {
            distanceToTarget = limelight.getDistance() + Constants.GOAL_RADIUS;
        } else {
            Pose2d currentPose = robotTracker.getLatencyCompedPoseMeters();
            Translation2d relativeRobotPosition = currentPose.getTranslation().minus(Constants.GOAL_POSITION);
            distanceToTarget = relativeRobotPosition.getNorm();
        }

        ShooterPreset shooterPreset = visionLookUpTable.getShooterPreset(distanceToTarget);
        shooter.setShooterSpeed(shooterPreset.getFlywheelSpeed());
        shooter.setHoodPosition(shooterPreset.getHoodEjectAngle() + shooterHoodAngleBias);
    }

    /**
     * Calculates and sets the flywheel speed considering a moving robot
     */
    public void updateShooterState() {
        ballPredictionMode = BallPredictionMode.VELOCITY_COMPENSATED;

        MutableTranslation2d predictedPose = predictTranslationAtZeroVelocity(robotTracker.getLatencyCompedChassisSpeeds(),
                robotTracker.getLatencyCompedPoseMeters().getTranslation());
        double distanceToTarget = predictedPose.minus(Constants.GOAL_POSITION).getNorm();

        ShooterPreset shooterPreset = visionLookUpTable.getShooterPreset(distanceToTarget);
        shooter.setShooterSpeed(shooterPreset.getFlywheelSpeed());
        shooter.setHoodPosition(shooterPreset.getHoodEjectAngle() + shooterHoodAngleBias);
    }

    private final ArrayList<Object> forceVisionOn = new ArrayList<>(5);

    /**
     * Forces the vision system to be on.
     *
     * @param source The source of the call. Used to keep track of what is calling this method. Only once all sources are removed
     *               will vision be turned off.
     */
    public void forceVisionOn(Object source) {
        forceVisionOn.add(source);
        limelight.setCamMode(CamMode.VISION_PROCESSOR);
        limelight.setLedMode(LedMode.ON);
    }

    /**
     * Removes a source from the list of sources that are forcing vision on. Will turn vision off if the sources list is empty.
     *
     * @param source The source to remove.
     */
    public void unForceVisionOn(Object source) {
        forceVisionOn.remove(source);
        if (forceVisionOn.isEmpty()) {
            limelight.setCamMode(CamMode.DRIVER_CAMERA);
            limelight.setLedMode(LedMode.OFF);
        }
    }

    public boolean isVisionForcedOn() {
        return !forceVisionOn.isEmpty();
    }

    @Override
    public void update() {
        Pose2d robotTrackerPose = robotTracker.getLatencyCompedPoseMeters();
        Translation2d goalTranslationOffset = robotTrackerPose.getTranslation()
                .plus(Constants.LIMELIGHT_CENTER_OFFSET.rotateBy(getLatencyCompedLimelightRotation()))
                .minus(Constants.GOAL_POSITION);

        double angleToTarget = Math.atan2(goalTranslationOffset.getY(), goalTranslationOffset.getX());


        if (isVisionForcedOn() || Math.abs(angleToTarget - robotTrackerPose.getRotation().getRadians()) < Math.toRadians(50)) {
            limelight.setCamMode(CamMode.VISION_PROCESSOR);
            limelight.setLedMode(LedMode.ON);

            if (limelight.isTargetVisible()) {
                MutableTranslation2d robotTranslation = getCurrentTranslation();

                if (MathUtil.dist2(robotTracker.getLatencyCompedPoseMeters().getTranslation(),
                        robotTranslation) < Constants.VISION_MANAGER_DISTANCE_THRESHOLD_SQUARED) {

                    Pose2d visionPose = new Pose2d(robotTranslation.getTranslation2d(), getLatencyCompedLimelightRotation());
                    robotTracker.addVisionMeasurement(visionPose, getLimelightTime());

                    logData("Vision Pose X", visionPose.getX());
                    logData("Vision Pose Y", visionPose.getY());
                    logData("Vision Pose Angle", visionPose.getRotation().getDegrees());
                    logData("Vision Pose Time", getLimelightTime());
                    logData("Using Vision Info", "Using Vision Info");
                } else {
                    logData("Using Vision Info", "Position is too far from expected");
                }
            } else {
                logData("Using Vision Info", "No target visible");
            }
        } else {
            limelight.setCamMode(CamMode.VISION_PROCESSOR);
            limelight.setLedMode(LedMode.ON);
            logData("Using Vision Info", "Not pointing at target");
        }
    }


    @Override
    public void close() throws Exception {

    }
}
