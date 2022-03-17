package frc.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.subsystem.Drive.DriveState;
import frc.subsystem.Hopper.HopperState;
import frc.utility.ControllerDriveInputs;
import frc.utility.Limelight;
import frc.utility.Limelight.LedMode;
import frc.utility.MathUtil;
import frc.utility.Timer;
import frc.utility.shooter.visionlookup.ShooterConfig;
import frc.utility.shooter.visionlookup.VisionLookUpTable;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

import java.util.HashSet;
import java.util.Optional;
import java.util.Set;

import static frc.robot.Constants.MAX_SHOOT_SPEED;
import static frc.utility.geometry.GeometryUtils.angleOf;

public final class VisionManager extends AbstractSubsystem {
    private static final @NotNull VisionManager instance = new VisionManager();

    private final @NotNull RobotTracker robotTracker = RobotTracker.getInstance();
    private final @NotNull Limelight limelight = Limelight.getInstance();
    private final @NotNull Drive drive = Drive.getInstance();
    private final @NotNull Shooter shooter = Shooter.getInstance();

    public final VisionLookUpTable visionLookUpTable = VisionLookUpTable.getInstance();

    public void setShooterConfig(ShooterConfig shooterConfig) {
        visionLookUpTable.setShooterConfig(shooterConfig);
    }

    private VisionManager() {
        super(Constants.VISION_MANAGER_PERIOD, 2);
    }

    public static @NotNull VisionManager getInstance() {
        return instance;
    }

    @Override
    public void selfTest() {

    }

    @Override
    public void logData() {
        logData("Distance to Target", Units.metersToInches(getDistanceToTarget()));
        logData("Rotation Target", getAngleToTarget().getDegrees());
        logData("Allowed Turn Error", getAllowedTurnError());

        logData("Allow Shooting Robot Speed", drive.getSpeedSquared() < Constants.MAX_SHOOT_SPEED_SQUARED);
        logData("Is Robot Allowed Shoot Aiming", !drive.isAiming());
        logData("Is Robot Allowed Shoot Tilt",
                Math.abs(robotTracker.getGyro().getRoll()) < 3 && Math.abs(robotTracker.getGyro().getPitch()) < 3);
    }

    public void shootAndMove(ControllerDriveInputs controllerDriveInputs, boolean useFieldRelative) {
        Translation2d robotVelocity = getRobotVel();
        Translation2d aimToPosition = getVelocityAdjustedRelativeTranslation(
                predictFutureTranslation(0.1, getRelativeGoalTranslation(), robotVelocity),
                robotVelocity
        );

        double targetAngle = angleOf(aimToPosition).getRadians();

        // Get the angle that will be used in the future to calculate the end velocity of the turn
        Translation2d futureAimToPosition = getVelocityAdjustedRelativeTranslation(
                predictFutureTranslation(0.2, getRelativeGoalTranslation(), robotVelocity),
                robotVelocity
        );
        double futureTargetAngle = angleOf(futureAimToPosition).getRadians();

        drive.updateTurn(controllerDriveInputs, new State(targetAngle, (futureTargetAngle - targetAngle) * 10), useFieldRelative,
                getAllowedTurnError());

        updateShooterState(aimToPosition.getNorm());

        tryToShoot(false);
    }


    public void autoTurnRobotToTarget(ControllerDriveInputs controllerDriveInputs, boolean fieldRelative) {
        drive.updateTurn(controllerDriveInputs, getAngleToTarget(), fieldRelative, getAllowedTurnError());

        updateShooterState(getDistanceToTarget());
        tryToShoot(true);
    }

    private void tryToShoot(boolean doSpeedCheck) {
        if ((!drive.isAiming()) && (drive.getSpeedSquared() < Constants.MAX_SHOOT_SPEED_SQUARED || !doSpeedCheck) &&
                Math.abs(robotTracker.getGyro().getRoll()) < 3 && Math.abs(robotTracker.getGyro().getPitch()) < 3) {
            shooter.setFiring(limelight.isTargetVisible() || DriverStation.isAutonomous());
            if (shooter.isFiring()) {
                if (!checksPassedLastTime && lastPrintTime + 0.5 < Timer.getFPGATimestamp()) {
                    lastPrintTime = Timer.getFPGATimestamp();
                    checksPassedLastTime = true;
                    System.out.println(
                            "Shooting at " + (150 - DriverStation.getMatchTime()) + " "
                                    + visionLookUpTable.getShooterPreset(Units.metersToInches(getDistanceToTarget())));
                }
            } else {
                lastChecksFailedTime = Timer.getFPGATimestamp();
                checksPassedLastTime = false;
            }
        } else {
            checksPassedLastTime = false;
            lastChecksFailedTime = Timer.getFPGATimestamp();
        }

        Hopper.getInstance().setHopperState(HopperState.ON);

        logData("Last Shooter Checks Failed Time", Timer.getFPGATimestamp() - lastChecksFailedTime);
    }

    /**
     * Updates the shooter state based on the distance to the target
     */
    public void updateShooterState() {
        updateShooterState(getDistanceToTarget());
    }

    /**
     * Updates the shooter state based on the distance to the target
     *
     * @param distanceToTarget the distance to the target (in meters)
     */
    public void updateShooterState(double distanceToTarget) {
        shooter.set(visionLookUpTable.getShooterPreset(Units.metersToInches(distanceToTarget)));
    }

    /**
     * @return the current robot velocity from the robot tracker
     */
    @Contract(pure = true)
    public Translation2d getRobotVel() {
        ChassisSpeeds chassisSpeeds = robotTracker.getLatencyCompedChassisSpeeds();
        return new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    }

    /**
     * @return the current translation of the robot based on the vision data. Will only give correct results if the limelight can
     * see the target
     */
    @Contract(pure = true)
    private @NotNull Optional<Translation2d> getVisionTranslation() {
        if (!limelight.isTargetVisible()) return Optional.empty();

        Rotation2d currentGyroAngle = getLatencyCompedLimelightRotation();

        double distanceToTarget = limelight.getDistanceM() + Constants.GOAL_RADIUS + Units.inchesToMeters(23);
        double angleToTarget = currentGyroAngle.getDegrees() - limelight.getHorizontalOffset();
        return Optional.of(new Translation2d(distanceToTarget * Math.cos(Math.toRadians(angleToTarget)),
                distanceToTarget * Math.sin(Math.toRadians(angleToTarget)))
                .minus(Constants.LIMELIGHT_CENTER_OFFSET.rotateBy(currentGyroAngle)).plus(Constants.GOAL_POSITION));
    }

    private Translation2d getRelativeGoalTranslation() {
        return robotTracker.getLatencyCompedPoseMeters().getTranslation()
                .plus(robotPositionOffset)
                .minus(Constants.GOAL_POSITION);
    }

    /**
     * Adds a vision update to the robot tracker even if the calculated pose is too far from the expected pose.
     * <p>
     * You need to call {@link #forceVisionOn(Object)} before calling this method.
     */
    public void forceUpdatePose() {
        limelight.setLedMode(LedMode.ON);
        Optional<Translation2d> visionTranslation = getVisionTranslation();
        visionTranslation.ifPresent(
                mutableTranslation2d ->
                        robotTracker.addVisionMeasurement(
                                new Pose2d(mutableTranslation2d, getLatencyCompedLimelightRotation()),
                                getLimelightTime())
        );
    }

    /**
     * @return the angle the robot needs to face to point towards the target
     */
    public Rotation2d getAngleToTarget() {
        return angleOf(getRelativeGoalTranslation());
    }

    /**
     * @return Distance to the target in meters
     */
    public double getDistanceToTarget() {
        return getRelativeGoalTranslation().getNorm();
    }


    double lastChecksFailedTime = 0;
    double lastPrintTime = 0;
    boolean checksPassedLastTime = false;
    private @NotNull Translation2d robotPositionOffset = new Translation2d(0, 0);

    /**
     * {@code Math.tan(Constants.GOAL_RADIUS / getDistanceToTarget())}
     *
     * @return The allowed turn error in radians
     */
    private double getAllowedTurnError() {
        return Math.tan((Constants.GOAL_RADIUS * 0.8) / getDistanceToTarget());
    }

    @Contract(pure = true)
    public @NotNull Rotation2d getLatencyCompedLimelightRotation() {
        return robotTracker.getGyroRotation(getLimelightTime());
    }

    /**
     * @return the time of the last vision update in seconds
     */
    @Contract(pure = true)
    private double getLimelightTime() {
        double latency = Timer.getFPGATimestamp(); //- (limelight.getLatency() / 1000.0) - (11.0 / 1000);
        logData("Limelight Latency", (limelight.getLatency() / 1000) + (11.0 / 1000));
        return latency;
    }

    private final Set<Object> forceVisionOn = new HashSet<>(5);

    /**
     * Forces the vision system to be on.
     *
     * @param source The source of the call. Used to keep track of what is calling this method. Only once all sources are removed
     *               will vision be turned off.
     */
    public void forceVisionOn(Object source) {
        synchronized (forceVisionOn) {
            forceVisionOn.add(source);
        }
        limelight.setLedMode(LedMode.ON);
    }

    /**
     * Removes a source from the list of sources that are forcing vision on. Will turn vision off if the sources list is empty.
     *
     * @param source The source to remove.
     */
    public void unForceVisionOn(Object source) {
        synchronized (forceVisionOn) {
            forceVisionOn.remove(source);
            if (forceVisionOn.isEmpty()) {
                limelight.setLedMode(LedMode.OFF);
            }
        }
    }

    public boolean isVisionForcedOn() {
        return !forceVisionOn.isEmpty();
    }

    private final Object updateLoopSource = new Object();

    @Override
    public void update() {
        Pose2d robotTrackerPose = robotTracker.getLatencyCompedPoseMeters();
        Translation2d relativeGoalPos = robotTrackerPose.getTranslation()
                .plus(Constants.LIMELIGHT_CENTER_OFFSET.rotateBy(getLatencyCompedLimelightRotation()))
                .minus(Constants.GOAL_POSITION);

        double angleToTarget = Math.atan2(relativeGoalPos.getY(), relativeGoalPos.getX());


        if (Math.abs(angleToTarget - robotTrackerPose.getRotation().getRadians()) < Math.toRadians(50)) {
            forceVisionOn(updateLoopSource);
        } else {
            unForceVisionOn(updateLoopSource);
        }


        Optional<Translation2d> robotTranslationOptional = getVisionTranslation();
        if (robotTranslationOptional.isPresent()) {
            Translation2d robotTranslation = robotTranslationOptional.get();

            Pose2d visionPose = new Pose2d(robotTranslation, getLatencyCompedLimelightRotation());
            logData("Vision Pose X", visionPose.getX());
            logData("Vision Pose Y", visionPose.getY());
            logData("Vision Pose Angle", visionPose.getRotation().getDegrees());
            logData("Vision Pose Time", getLimelightTime());

            //TODO: Check that the contours aren't touching the edge of the screen before using them
            if (MathUtil.dist2(robotTracker.getLatencyCompedPoseMeters().getTranslation(),
                    robotTranslation) < Constants.VISION_MANAGER_DISTANCE_THRESHOLD_SQUARED) {

                robotPositionOffset = robotTranslation.minus(robotTracker.getLatencyCompedPoseMeters().getTranslation());
                logData("Using Vision Info", "Using Vision Info");
            } else {
                logData("Using Vision Info", "Position is too far from expected");
            }
        } else {
            logData("Using Vision Info", "No target visible");
        }
    }


    private static final ControllerDriveInputs CONTROLLER_DRIVE_NO_MOVEMENT = new ControllerDriveInputs(0, 0, 0);

    /**
     * For auto use only
     */
    @SuppressWarnings({"unused", "BusyWait"})
    public void shootBalls(double numBalls) throws InterruptedException {
        forceVisionOn(this);
        if (drive.driveState == DriveState.RAMSETE) {
            drive.setAutoAiming(true);
        } else {
            autoTurnRobotToTarget(CONTROLLER_DRIVE_NO_MOVEMENT, true);
        }
        updateShooterState();

        while ((drive.isAiming() || !shooter.isHoodAtTargetAngle() || !shooter.isShooterAtTargetSpeed() || drive.getSpeedSquared() > MAX_SHOOT_SPEED)) {
            if (drive.driveState == DriveState.RAMSETE) {
                drive.setAutoAiming(true);
            } else {
                autoTurnRobotToTarget(CONTROLLER_DRIVE_NO_MOVEMENT, true);
            }
            updateShooterState();
            Thread.sleep(10); // Will exit if interrupted
        }
        double shootUntilTime = Timer.getFPGATimestamp() + (numBalls * Constants.SHOOT_TIME_PER_BALL);

        shooter.setFiring(true);
        while (Timer.getFPGATimestamp() < shootUntilTime) {
            if (drive.driveState == DriveState.RAMSETE) {
                drive.setAutoAiming(true);
            } else {
                autoTurnRobotToTarget(CONTROLLER_DRIVE_NO_MOVEMENT, true);
            }
            updateShooterState();
            Thread.sleep(10); // Will exit if interrupted
        }
        unForceVisionOn(this);
        shooter.setFiring(false);
        shooter.setSpeed(0);
        drive.setAutoAiming(false);
    }


    @Override
    public void close() throws Exception {

    }


    //------- Math Methods -------

    /**
     * @return the current position of the robot based on a translation and some time. It adds the current velocity * time to the
     * translation.
     */
    @Contract(pure = true)
    private Translation2d predictFutureTranslation(double predictAheadTime, Translation2d currentTranslation,
                                                   Translation2d currentVelocity) {
        return currentTranslation.plus(currentVelocity.times(predictAheadTime));
    }

    /**
     * Calculates a "fake" target position that the robot should aim to based on the current position and the current velocity. If
     * the robot shoots to this "fake" position, it will shoot the ball into the actual target.
     *
     * @param relativeGoalTranslation The relative position of the target from the robot
     * @param robotVelocity           The velocity of the robot
     * @return The position of the "fake" target
     */
    @Contract(pure = true)
    private @NotNull Translation2d getVelocityAdjustedRelativeTranslation(
            @NotNull Translation2d relativeGoalTranslation, @NotNull Translation2d robotVelocity) {
        return relativeGoalTranslation.minus(robotVelocity); //TODO: Change to an actual equation
    }
}
