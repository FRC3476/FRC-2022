package frc.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.subsystem.Drive.DriveState;
import frc.subsystem.Hopper.HopperState;
import frc.utility.ControllerDriveInputs;
import frc.utility.Limelight;
import frc.utility.Limelight.LedMode;
import frc.utility.MathUtil;
import frc.utility.Timer;
import frc.utility.geometry.MutableTranslation2d;
import frc.utility.shooter.visionlookup.ShooterConfig;
import frc.utility.shooter.visionlookup.ShooterPreset;
import frc.utility.shooter.visionlookup.VisionLookUpTable;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;

import static frc.utility.OrangeUtility.getSpeed;
import static frc.utility.OrangeUtility.getTranslation2d;

public final class VisionManager extends AbstractSubsystem {
    private static final @NotNull VisionManager instance = new VisionManager();

    private final @NotNull RobotTracker robotTracker = RobotTracker.getInstance();
    private final @NotNull Limelight limelight = Limelight.getInstance();
    private final @NotNull Drive drive = Drive.getInstance();
    private final @NotNull Shooter shooter = Shooter.getInstance();

    public final VisionLookUpTable visionLookUpTable = VisionLookUpTable.getInstance();
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

    private VisionManager() {
        super(Constants.VISION_MANAGER_PERIOD);
    }

    public static @NotNull VisionManager getInstance() {
        return instance;
    }

    @Override
    public void selfTest() {

    }

    @Override
    public void logData() {

    }

    public void shootAndMove(ControllerDriveInputs controllerDriveInputs, boolean useFieldRelative) {
        autoTurnRobotToTarget(controllerDriveInputs, useFieldRelative);
//        Rotation2d targetRotation = getPredictedRotationTarget();
//        drive.updateTurn(controllerDriveInputs, targetRotation, useFieldRelative);
//        shooter.setFiring(!drive.isAiming());
    }

    public Rotation2d getPredictedRotationTarget() {
        MutableTranslation2d relativeRobotPosition = predictTranslationAtZeroVelocity(
                robotTracker.getLatencyCompedChassisSpeeds(),
                robotTracker.getLatencyCompedPoseMeters().getTranslation()).minus(Constants.GOAL_POSITION);
        return new Rotation2d(Math.atan2(relativeRobotPosition.getY(), relativeRobotPosition.getX()));
    }

    /**
     * @return the current translation of the robot based on the vision data
     */
    @Contract(pure = true)
    private MutableTranslation2d getCurrentTranslation() {
        Rotation2d currentGyroAngle = getLatencyCompedLimelightRotation();

        double distanceToTarget = limelight.getDistanceM() + Constants.GOAL_RADIUS;
        double angleToTarget = currentGyroAngle.getDegrees() - limelight.getHorizontalOffset();
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
        limelight.setLedMode(LedMode.ON);
        if (limelight.isTargetVisible()) {
            robotTracker.addVisionMeasurement(new Pose2d(getCurrentTranslation().getTranslation2d(),
                    getLatencyCompedLimelightRotation()), getLimelightTime());
        }
    }

    public Rotation2d getAngleOfTarget() {
        if (limelight.isTargetVisible()) {
            double degreeOffset = Limelight.getInstance().getHorizontalOffset();
            return robotTracker.getGyroAngle().minus(Rotation2d.fromDegrees(degreeOffset));
        } else {
            //Use best guess if no target is visible
            Translation2d relativeRobotPosition = robotTracker.getLatencyCompedPoseMeters().getTranslation()
                    .minus(Constants.GOAL_POSITION);
            return new Rotation2d(Math.atan2(relativeRobotPosition.getY(), relativeRobotPosition.getX()));

            //drive.swerveDriveFieldRelative(controllerDriveInputs);
        }
    }

    double bypassAimCheckUntil = 0;

    public void autoTurnRobotToTarget(ControllerDriveInputs controllerDriveInputs, boolean fieldRelative) {
        logData("Allow Shooting Robot Speed", drive.getSpeedSquared() < Constants.MAX_SHOOT_SPEED_SQUARED);
        logData("Is Robot Allowed Shoot Aiming", !drive.isAiming());

        drive.updateTurn(controllerDriveInputs, getAngleOfTarget(), fieldRelative);

        if (drive.getSpeedSquared() > Constants.MAX_SHOOT_SPEED_SQUARED) {
            bypassAimCheckUntil = 0;
        }
        if ((!drive.isAiming() || Timer.getFPGATimestamp() < bypassAimCheckUntil)
                && drive.getSpeedSquared() < Constants.MAX_SHOOT_SPEED_SQUARED) {
            bypassAimCheckUntil = Timer.getFPGATimestamp() + 0.1;
            shooter.setFiring(true);
        } else {
            bypassAimCheckUntil = 0;
        }
        Hopper.getInstance().setHopperState(HopperState.ON);
    }

    public Rotation2d getLatencyCompedLimelightRotation() {
        return robotTracker.getGyroRotation(getLimelightTime());
    }

    /**
     * @return the time of the last vision update in seconds
     */
    private double getLimelightTime() {
        return Timer.getFPGATimestamp() - (limelight.getLatency() / 1000) - 0.011;
    }


    NetworkTableEntry visionDistanceOffset = NetworkTableInstance.getDefault().getEntry("VisionDistanceOffset");

    {
        visionDistanceOffset.setDouble(Constants.GOAL_RADIUS - 18);
    }

    /**
     * Calculates and sets the flywheel speed considering a static robot velocity
     */
    public void updateShooterStateStaticPose() {
        double distanceToTarget;
        if (limelight.isTargetVisible()) {
            distanceToTarget = limelight.getDistanceM() + Constants.GOAL_RADIUS;
        } else {
            Pose2d currentPose = robotTracker.getLatencyCompedPoseMeters();
            Translation2d relativeRobotPosition = currentPose.getTranslation().minus(Constants.GOAL_POSITION);
            distanceToTarget = relativeRobotPosition.getNorm();
        }

        logData("Distance to Target", Units.metersToInches(distanceToTarget));

        ShooterPreset shooterPreset = visionLookUpTable.getShooterPreset(Units.metersToInches(distanceToTarget));
        shooter.setSpeed(shooterPreset.getFlywheelSpeed());
        shooter.setHoodPosition(shooterPreset.getHoodEjectAngle() + shooterHoodAngleBias);
    }

    /**
     * Calculates and sets the flywheel speed considering a moving robot
     */
    public void updateShooterState() {
        updateShooterStateStaticPose();
//        MutableTranslation2d predictedPose = predictTranslationAtZeroVelocity(robotTracker.getLatencyCompedChassisSpeeds(),
//                robotTracker.getLatencyCompedPoseMeters().getTranslation());
//        double distanceToTarget = predictedPose.minus(Constants.GOAL_POSITION).getNorm();
//
//        ShooterPreset shooterPreset = visionLookUpTable.getShooterPreset(distanceToTarget);
//        shooter.setSpeed(shooterPreset.getFlywheelSpeed());
//        shooter.setHoodPosition(shooterPreset.getHoodEjectAngle() + shooterHoodAngleBias);
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
            limelight.setLedMode(LedMode.OFF);
        }
    }

    public boolean isVisionForcedOn() {
        return !forceVisionOn.isEmpty();
    }

    private final Object updateLoopSource = new Object();

    @Override
    public void update() {
        Pose2d robotTrackerPose = robotTracker.getLatencyCompedPoseMeters();
        Translation2d goalTranslationOffset = robotTrackerPose.getTranslation()
                .plus(Constants.LIMELIGHT_CENTER_OFFSET.rotateBy(getLatencyCompedLimelightRotation()))
                .minus(Constants.GOAL_POSITION);

        double angleToTarget = Math.atan2(goalTranslationOffset.getY(), goalTranslationOffset.getX());


        if (Math.abs(angleToTarget - robotTrackerPose.getRotation().getRadians()) < Math.toRadians(50)) {
            forceVisionOn(updateLoopSource);
        } else {
            unForceVisionOn(updateLoopSource);
        }

        if (limelight.isTargetVisible()) {
            MutableTranslation2d robotTranslation = getCurrentTranslation();

            Pose2d visionPose = new Pose2d(robotTranslation.getTranslation2d(), getLatencyCompedLimelightRotation());
            logData("Vision Pose X", visionPose.getX());
            logData("Vision Pose Y", visionPose.getY());
            logData("Vision Pose Angle", visionPose.getRotation().getDegrees());
            logData("Vision Pose Time", getLimelightTime());

            if (MathUtil.dist2(robotTracker.getLatencyCompedPoseMeters().getTranslation(),
                    robotTranslation) < Constants.VISION_MANAGER_DISTANCE_THRESHOLD_SQUARED) {
                //robotTracker.addVisionMeasurement(visionPose, getLimelightTime());
                logData("Using Vision Info", "Using Vision Info");
            } else {
                logData("Using Vision Info", "Position is too far from expected");
            }
        } else {
            logData("Using Vision Info", "No target visible");
        }
    }


    private static final ControllerDriveInputs CONTROLLER_DRIVE_NO_MOVEMENT = new ControllerDriveInputs(0, 0, 0);

    public volatile boolean killAuto = false;

    /**
     * For auto use only
     */
    @SuppressWarnings("unused")
    public void shootBalls(double numBalls) {
        forceVisionOn(this);
        if (drive.driveState == DriveState.RAMSETE) {
            drive.setAutoAiming(true);
        } else {
            autoTurnRobotToTarget(CONTROLLER_DRIVE_NO_MOVEMENT, true);
        }
        updateShooterState();

        while ((drive.isAiming() || !shooter.isHoodAtTargetAngle() || !shooter.isShooterAtTargetSpeed() || drive.getSpeedSquared() > 0.1) && !killAuto) {
            if (drive.driveState == DriveState.RAMSETE) {
                drive.setAutoAiming(true);
            } else {
                autoTurnRobotToTarget(CONTROLLER_DRIVE_NO_MOVEMENT, true);
            }
            updateShooterState();
            Thread.onSpinWait();
        }
        double shootUntilTime = Timer.getFPGATimestamp() + (numBalls * Constants.SHOOT_TIME_PER_BALL);

        shooter.setFiring(true);
        while (Timer.getFPGATimestamp() < shootUntilTime && !killAuto) {
            if (drive.driveState == DriveState.RAMSETE) {
                drive.setAutoAiming(true);
            } else {
                autoTurnRobotToTarget(CONTROLLER_DRIVE_NO_MOVEMENT, true);
            }
            updateShooterState();
            Thread.onSpinWait();
        }
        unForceVisionOn(this);
        shooter.setFiring(false);
        shooter.setSpeed(0);
        drive.setAutoAiming(false);
    }

    /**
     * For auto use only
     */
    @SuppressWarnings("unused")
    public void shootBalls(double numBalls, double flywheelSpeed, double ejectionAngle) {
        forceVisionOn(this);
        if (drive.driveState == DriveState.RAMSETE) {
            drive.setAutoAiming(true);
        } else {
            autoTurnRobotToTarget(CONTROLLER_DRIVE_NO_MOVEMENT, true);
        }
        shooter.setSpeed(flywheelSpeed);
        shooter.setHoodPosition(ejectionAngle);

        while ((drive.isAiming() || !shooter.isFiring() || drive.getSpeedSquared() > 0.1) && !killAuto) {
            if (drive.driveState == DriveState.RAMSETE) {
                drive.setAutoAiming(true);
            } else {
                autoTurnRobotToTarget(CONTROLLER_DRIVE_NO_MOVEMENT, true);
            }
            Thread.yield();
        }
        double shootUntilTime = numBalls * Constants.SHOOT_TIME_PER_BALL;

        shooter.setFiring(true);
        while (Timer.getFPGATimestamp() < shootUntilTime && !killAuto) {
            if (drive.driveState == DriveState.RAMSETE) {
                drive.setAutoAiming(true);
            } else {
                autoTurnRobotToTarget(CONTROLLER_DRIVE_NO_MOVEMENT, true);
            }
            Thread.yield();
        }
        unForceVisionOn(this);
        shooter.setFiring(false);
        shooter.setSpeed(0);
        drive.setAutoAiming(false);
    }


    @Override
    public void close() throws Exception {

    }
}
