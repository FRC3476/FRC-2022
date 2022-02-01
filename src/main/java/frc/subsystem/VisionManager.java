package frc.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

public final class VisionManager extends AbstractSubsystem {
    private static VisionManager instance = new VisionManager();

    RobotTracker robotTracker = RobotTracker.getInstance();
    Limelight limelight = Limelight.getInstance();
    Drive drive = Drive.getInstance();
    Shooter shooter = Shooter.getInstance();

    private final VisionLookUpTable visionLookUpTable = VisionLookUpTable.getInstance();

    public void setShooterConfig(ShooterConfig shooterConfig) {
        visionLookUpTable.setShooterConfig(shooterConfig);
    }

    public enum BallPredictionMode {
        VELOCITY_COMPENSATED, STATIC_POSE
    }

    public BallPredictionMode getBallPredictionMode() {
        return ballPredictionMode;
    }

    private BallPredictionMode ballPredictionMode;

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
    public void forceUpdatePose() {
        limelight.setCamMode(CamMode.VISION_PROCESSOR);
        limelight.setLedMode(LedMode.ON);
        if (limelight.isTargetVisible()) {
            robotTracker.resetPosition(new Pose2d(getCurrentTranslation().getTranslation2d(),
                    robotTracker.getLatencyCompedPoseMeters().getRotation()));
        }
    }

    public void autoTurnRobotToTarget(ControllerDriveInputs controllerDriveInputs, boolean fieldRelative) {
        double degreeOffset;
        if (limelight.isTargetVisible()) {
            degreeOffset = Limelight.getInstance().getHorizontalOffset();
        } else {
            //Use best guess if no target is visible
            Translation2d relativeRobotPosition = robotTracker.getLatencyCompedPoseMeters().getTranslation().minus(
                    Constants.GOAL_POSITION);
            degreeOffset = Math.toDegrees(Math.atan2(relativeRobotPosition.getY(), relativeRobotPosition.getX()));
        }
        drive.updateTurn(controllerDriveInputs.getX(), controllerDriveInputs.getY(),
                robotTracker.getGyroAngle().rotateBy(Rotation2d.fromDegrees(-degreeOffset)), fieldRelative);

        shooter.setFiring(!drive.isAiming());
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
        shooter.setHoodPosition(shooterPreset.getHoodEjectAngle());
    }

    /**
     * Calculates and sets the flywheel speed considering a moving robot
     */
    public void updateShooterState() {
        ballPredictionMode = BallPredictionMode.VELOCITY_COMPENSATED;
    }

    private boolean forceVisionOn = false;

    public void forceVisionOn(boolean visionOn) {
        forceVisionOn = visionOn;
        if (visionOn) {
            limelight.setCamMode(CamMode.VISION_PROCESSOR);
            limelight.setLedMode(LedMode.ON);
        }
    }

    @Override
    public void update() {
        Pose2d robotTrackerPose = robotTracker.getLatencyCompedPoseMeters();
        Translation2d goalTranslationOffset = robotTrackerPose.getTranslation()
                .minus(Constants.LIMELIGHT_CENTER_OFFSET.rotateBy(robotTrackerPose.getRotation()))
                .minus(Constants.GOAL_POSITION);

        double angleToTarget = Math.atan2(goalTranslationOffset.getY(), goalTranslationOffset.getX());


        if (forceVisionOn || Math.abs(angleToTarget - robotTrackerPose.getRotation().getRadians()) < Math.toRadians(50)) {
            limelight.setCamMode(CamMode.VISION_PROCESSOR);
            limelight.setLedMode(LedMode.ON);

            if (limelight.isTargetVisible()) {
                MutableTranslation2d robotTranslation = getCurrentTranslation();
                if (MathUtil.dist2(robotTracker.getLatencyCompedPoseMeters().getTranslation(),
                        robotTranslation) < Constants.VISION_MANAGER_DISTANCE_THRESHOLD_SQUARED) {
                    robotTracker.addVisionMeasurement(
                            new Pose2d(robotTranslation.getTranslation2d(),
                                    robotTracker.getLatencyCompedPoseMeters().getRotation()),
                            Timer.getFPGATimestamp() - limelight.getLatency() - 0.011);
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
