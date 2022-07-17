package frc.subsystem;

import com.dacubeking.AutoBuilder.robot.annotations.AutoBuilderAccessible;
import com.dacubeking.AutoBuilder.robot.drawable.Circle;
import com.dacubeking.AutoBuilder.robot.drawable.Renderer;
import com.dacubeking.AutoBuilder.robot.sender.pathpreview.RobotPositionSender;
import com.dacubeking.AutoBuilder.robot.sender.pathpreview.RobotState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.subsystem.BlinkinLED.BlinkinLedMode;
import frc.subsystem.BlinkinLED.LedStatus;
import frc.subsystem.Drive.DriveState;
import frc.subsystem.Hopper.HopperState;
import frc.subsystem.Shooter.FeederWheelState;
import frc.utility.ControllerDriveInputs;
import frc.utility.Limelight;
import frc.utility.Limelight.LedMode;
import frc.utility.Timer;
import frc.utility.geometry.MutableTranslation2d;
import frc.utility.shooter.visionlookup.ShooterConfig;
import frc.utility.shooter.visionlookup.VisionLookUpTable;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.HashSet;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Function;

import static frc.robot.Constants.*;
import static frc.utility.MathUtil.dist2;
import static frc.utility.geometry.GeometryUtils.angleOf;

public final class VisionManager extends AbstractSubsystem {
    private static final Color8Bit LIGHT_BLUE = new Color8Bit(36, 191, 212);
    private static final ReentrantReadWriteLock VISION_MANGER_INSTANCE_LOCK = new ReentrantReadWriteLock();

    private static @Nullable VisionManager instance = null;

    private final @NotNull Limelight limelight = Limelight.getInstance();

    public final @NotNull VisionLookUpTable visionLookUpTable = VisionLookUpTable.getInstance();

    public void setShooterConfig(ShooterConfig shooterConfig) {
        visionLookUpTable.setShooterConfig(shooterConfig);
    }

    private VisionManager() {
        super(Constants.VISION_MANAGER_PERIOD, 1);
        logData("IS VISION GOOD", true);
    }

    @AutoBuilderAccessible
    public static @NotNull VisionManager getInstance() {
        VISION_MANGER_INSTANCE_LOCK.readLock().lock();
        try {
            if (instance != null) {
                return instance;
            }
        } finally {
            VISION_MANGER_INSTANCE_LOCK.readLock().unlock();
        }

        VISION_MANGER_INSTANCE_LOCK.writeLock().lock();
        try {
            return Objects.requireNonNullElseGet(instance, () -> instance = new VisionManager());
        } finally {
            VISION_MANGER_INSTANCE_LOCK.writeLock().unlock();
        }
    }

    @Override
    public void selfTest() {

    }

    @Override
    public void logData() {
        final @NotNull RobotTracker robotTracker = RobotTracker.getInstance();
        final @NotNull Drive drive = Drive.getInstance();
        final @NotNull Shooter shooter = Shooter.getInstance();

        logData("Distance to Target", Units.metersToInches(getDistanceToTarget()));
        logData("Rotation Target", getAngleToTarget().getDegrees());

        Vector3D correctVector = limelight.getCorrectTargetVector();
        logData("New Distance", Math.hypot(correctVector.getX(), correctVector.getZ()));

        logData("Old Distance ", limelight.getDistance() + Constants.GOAL_RADIUS_IN + 23);

        Vector2d targetPx = limelight.getTargetPosInCameraPixels();

        logData("py", targetPx.y);
        logData("px", targetPx.x);

//        Vector2d newRelGoalPos = limelight.getCorrectGoalPos();
//        logData("New Z", newRelGoalPos.x);
//        logData("New X", newRelGoalPos.y);

        logData("Allow Shooting Robot Speed", drive.getSpeedSquared() < Constants.MAX_SHOOT_SPEED_SQUARED);
        logData("Is Robot Allowed Shoot Tilt",
                Math.abs(robotTracker.getGyro().getRoll()) < 3 && Math.abs(robotTracker.getGyro().getPitch()) < 3);

        Translation2d robotVelocity = getRobotVel();
        Translation2d relativeRobotTranslation = getRelativeGoalTranslation();
        logData("Relative Robot Translation X", relativeRobotTranslation.getX());
        logData("Relative Robot Translation Y", relativeRobotTranslation.getY());

        logData("Vision Robot Velocity X", robotVelocity.getX());
        logData("Vision Robot Velocity Y", robotVelocity.getY());
        logData("Time Shooting", 0);

        double timeFromLastShoot = Timer.getFPGATimestamp() - shooter.getLastShotTime();
        double shooterLookAheadTime = 0.15 - timeFromLastShoot;
        if (shooterLookAheadTime < 0) {
            shooterLookAheadTime = 0.15;
        }

        double turnDelay = 0.00;

        Translation2d aimToPosition = getAdjustedTranslation(shooterLookAheadTime + turnDelay);

        Translation2d fieldCentricCords =
                RobotTracker.getInstance().getLastEstimatedPoseMeters().getTranslation().minus(aimToPosition);
        logData("Calculated Target X", fieldCentricCords.getX());
        logData("Calculated Target Y", fieldCentricCords.getY());

        Renderer.render(new Circle((float) fieldCentricCords.getX(), (float) fieldCentricCords.getY(), 0.3f, LIGHT_BLUE));

        double allowedTurnError = getAllowedTurnError(aimToPosition.getNorm());

        logData("Allowed Turn Error", allowedTurnError);
    }

    public void shootAndMove(ControllerDriveInputs controllerDriveInputs, boolean useFieldRelative) {
        shootAndMove(controllerDriveInputs, useFieldRelative, true);
    }

    private final static Rotation2d ROTATION_OFFSET = Rotation2d.fromDegrees(1);

    /**
     * @param controllerDriveInputs The controller drive inputs to use
     * @param useFieldRelative      Whether to use field relative
     * @param sendDriveCommand      Whether to send the drive command to move the robot
     * @return The turn command to send to the drive subsystem
     */
    public State shootAndMove(ControllerDriveInputs controllerDriveInputs, boolean useFieldRelative, boolean sendDriveCommand) {
        final @NotNull Drive drive = Drive.getInstance();
        final @NotNull Shooter shooter = Shooter.getInstance();

        double timeFromLastShoot = Timer.getFPGATimestamp() - shooter.getLastShotTime();
        double shooterLookAheadTime = 0.15 - timeFromLastShoot;
        if (shooterLookAheadTime < 0) {
            shooterLookAheadTime = 0.15;
        }

        double turnDelay = 0.0;

        Translation2d aimToPosition = getAdjustedTranslation(shooterLookAheadTime + turnDelay).times(-1);
        double targetAngle = aimPointToDriveRotation(aimToPosition).plus(ROTATION_OFFSET).getRadians();

        // Get the angle that will be used in the future to calculate the end velocity of the turn
        Translation2d futureAimToPosition = getAdjustedTranslation(shooterLookAheadTime + turnDelay + 0.1).times(-1);
        double futureTargetAngle = aimPointToDriveRotation(futureAimToPosition).plus(ROTATION_OFFSET).getRadians();

        State turnGoal = new State(targetAngle, (futureTargetAngle - targetAngle) * 10);
        if (sendDriveCommand) {
            drive.updateTurn(controllerDriveInputs,
                    turnGoal,
                    useFieldRelative,
                    0);
        }


        Translation2d aimChecksPosition = getAdjustedTranslation(shooterLookAheadTime).times(-1);
        updateShooterState(aimChecksPosition.getNorm());

        tryToShoot(aimChecksPosition, (futureTargetAngle - targetAngle) * 10, false);
        return turnGoal;
    }


    public void autoTurnAndShoot(ControllerDriveInputs controllerDriveInputs, boolean fieldRelative) {
        final @NotNull Drive drive = Drive.getInstance();

        Optional<Translation2d> visionTranslation = getVisionTranslation();
        Translation2d aimPoint;
        if (visionTranslation.isPresent()) {
            // Use vision measurement directly cause vision doesn't update the robot tracker in auto.
            aimPoint = visionTranslation.get().minus(GOAL_POSITION).times(-1);
        } else {
            aimPoint = getRelativeGoalTranslation().times(-1);
        }

        drive.updateTurn(controllerDriveInputs, aimPointToDriveRotation(aimPoint).plus(ROTATION_OFFSET), fieldRelative,
                getAllowedTurnError());
        updateShooterState(aimPoint.getNorm());
        tryToShoot(aimPoint, 0, true);
    }

    public void stopAndShoot(ControllerDriveInputs controllerDriveInputs, boolean fieldRelative) {
        final @NotNull Drive drive = Drive.getInstance();
        Translation2d aimPoint = predictFutureTranslation(
                getRobotVel().getNorm() / drive.accelerationLimit.acceleration,
                getRelativeGoalTranslation(), getRobotVel(), getAccel()).times(-1);

        drive.updateTurn(controllerDriveInputs, aimPointToDriveRotation(aimPoint), fieldRelative, getAllowedTurnError());
        updateShooterState(aimPoint.getNorm());
        tryToShoot(aimPoint, 0, true);
    }

    private void tryToShoot(Translation2d aimPoint, double targetAngularSpeed, boolean doSpeedCheck) {
        final @NotNull RobotTracker robotTracker = RobotTracker.getInstance();
        final @NotNull Drive drive = Drive.getInstance();
        final @NotNull Shooter shooter = Shooter.getInstance();

        final boolean isAimed = Math.abs(aimPointToDriveRotation(aimPoint)
                .plus(ROTATION_OFFSET).minus(robotTracker.getGyroAngle()).getRadians()) < getAllowedTurnError(aimPoint.getNorm());

        final boolean isTurningSpeedCorrect =
                Math.abs(robotTracker.getLatencyCompedChassisSpeeds().omegaRadiansPerSecond - targetAngularSpeed)
                        < Math.toRadians(8);

        final boolean isUnderAccelLimit = getAccel().getNorm() < MAX_ACCELERATION_WHILE_SHOOTING;
        final boolean isStopped = (drive.getSpeedSquared() < Constants.MAX_SHOOT_SPEED_SQUARED || !doSpeedCheck);
        final boolean isFlatOnGround = (Math.abs(robotTracker.getGyro().getRoll()) < 3 &&
                Math.abs(robotTracker.getGyro().getPitch()) < 3) || IS_PRACTICE; //The roborio on the practice bot is tilted a
        // bit and cause this check to fail

        logData("Is allowed Shoot Turn Speed", isTurningSpeedCorrect);
        logData("Is Robot Allowed Shoot Aiming", isAimed);
        logData("Is Robot Allowed Shoot Acceleration", isUnderAccelLimit);

        final boolean hasBadVision = loopsWithBadVision.get() < Constants.MAX_BAD_VISION_ITERATIONS;


        //@formatter:off
        if (isAimed
                && isTurningSpeedCorrect
                && isUnderAccelLimit
                && isStopped
                && isFlatOnGround
                && hasBadVision) {
            //@formatter:on
            shooter.setFiring(true);
            if (shooter.isFiring()) {
                double time = Timer.getFPGATimestamp();
                if (!checksPassedLastTime && lastPrintTime + 1 < time) {
                    lastPrintTime = time;
                    checksPassedLastTime = true;
                    System.out.println(
                            "Shooting at " + (150 - DriverStation.getMatchTime()) + " Distance:  "
                                    + Units.metersToInches(aimPoint.getNorm()) + " "
                                    + "Accel: " + getAccel().getNorm());
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

        logData("Is Shooter Firing", shooter.isFiring());

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
        final @NotNull Shooter shooter = Shooter.getInstance();

        logData("Shooter Distance to Target", Units.metersToInches(distanceToTarget));
        shooter.set(visionLookUpTable.getShooterPreset(Units.metersToInches(distanceToTarget)));
    }

    /**
     * @return the current robot velocity from the robot tracker
     */
    @Contract(pure = true)
    public Translation2d getRobotVel() {
        ChassisSpeeds chassisSpeeds = RobotTracker.getInstance().getLatencyCompedChassisSpeeds();
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

        Vector3D offsetVector = limelight.getCorrectTargetVector();
        double angleOffset = Math.atan2(offsetVector.getX(), offsetVector.getZ());


        double distanceToTarget = Units.inchesToMeters(Math.hypot(offsetVector.getX(), offsetVector.getZ()));

        double angleToTarget = currentGyroAngle.getRadians() - angleOffset;
        return Optional.of(new Translation2d(distanceToTarget * Math.cos(angleToTarget),
                distanceToTarget * Math.sin(angleToTarget))
                .plus(GOAL_POSITION));
    }

    /**
     * @return field relative translation with the origin at the goal
     */
    private Translation2d getRelativeGoalTranslation() {
        final @NotNull RobotTracker robotTracker = RobotTracker.getInstance();
        return robotTracker.getLatencyCompedPoseMeters().getTranslation()
                .plus(robotPositionOffset)
                .minus(GOAL_POSITION);
    }

    /**
     * Adds a vision update to the robot tracker even if the calculated pose is too far from the expected pose.
     * <p>
     * You need to call {@link #forceVisionOn(Object)} before calling this method.
     */
    public void forceUpdatePose() {
        final @NotNull RobotTracker robotTracker = RobotTracker.getInstance();
        Optional<Translation2d> visionTranslation = getVisionTranslation();
        visionTranslation.ifPresent(
                translation2d -> {
                    loopsWithBadVision.set(0);
                    robotTracker.addVisionMeasurement(
                            translation2d,
                            getLimelightTime(), true);
                    robotPositionOffset = new Translation2d();
                }
        );
    }

    /**
     * @return the angle the robot needs to face to point towards the target
     */
    public Rotation2d getAngleToTarget() {
        return aimPointToDriveRotation(getRelativeGoalTranslation().times(-1)).plus(ROTATION_OFFSET);
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
        return getAllowedTurnError(getDistanceToTarget());
    }

    /**
     * {@code Math.tan(Constants.GOAL_RADIUS / getDistanceToTarget())}
     *
     * @return The allowed turn error in radians
     */
    private double getAllowedTurnError(double distance) {
        return Math.tan((Constants.GOAL_RADIUS * 0.5) / distance);
    }

    @Contract(pure = true)
    public @NotNull Rotation2d getLatencyCompedLimelightRotation() {
        final @NotNull RobotTracker robotTracker = RobotTracker.getInstance();
        return robotTracker.getGyroRotation(getLimelightTime());
    }

    /**
     * @return the time of the last vision update in seconds
     */
    @Contract(pure = true)
    private double getLimelightTime() {
        double limelightTime = Limelight.getInstance().getTimestamp();
        logData("Limelight Latency", Timer.getFPGATimestamp() - limelightTime);
        return limelightTime;
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

    private final LedStatus limelightUsingVisionStatus = new LedStatus(BlinkinLedMode.SOLID_LAWN_GREEN, 1);
    private final LedStatus limelightNotConnectedStatus = new LedStatus(BlinkinLedMode.SOLID_RED, 100);
    private final LedStatus limelightTooFarFromExpectedStatus = new LedStatus(BlinkinLedMode.SOLID_ORANGE, 100);
    private final LedStatus limelightNotVisibleStatus = new LedStatus(BlinkinLedMode.SOLID_RED_ORANGE, 100);

    private final AtomicInteger loopsWithBadVision = new AtomicInteger(0);

    @Override
    public void update() {
        final @NotNull RobotTracker robotTracker = RobotTracker.getInstance();
        final @NotNull BlinkinLED blinkinLED = BlinkinLED.getInstance();

        robotPositionOffset = new Translation2d();
        Translation2d relativeGoalPos = getRelativeGoalTranslation();

        double angleToTarget = Math.atan2(relativeGoalPos.getY(), relativeGoalPos.getX());

        if (limelight.isConnected()) {
            logData("Limelight Connected", true);
        } else {
            logData("Limelight Connected", false);
        }

        if (!limelight.isTargetVisible()) {
            blinkinLED.setStatus(limelightNotConnectedStatus);
        }

        if (Math.abs(new Rotation2d(angleToTarget).minus(robotTracker.getGyroAngle()).getRadians()) < Math.toRadians(50)) {
            forceVisionOn(updateLoopSource);
        } else {
            unForceVisionOn(updateLoopSource);
        }

        logData("Angle To Target", angleToTarget);


        Optional<Translation2d> robotTranslationOptional = getVisionTranslation();
        if (robotTranslationOptional.isPresent()) {
            Translation2d robotTranslation = robotTranslationOptional.get();

            Pose2d visionPose = new Pose2d(robotTranslation, getLatencyCompedLimelightRotation());
            logData("Vision Pose X", visionPose.getX());
            logData("Vision Pose Y", visionPose.getY());
            logData("Vision Pose Angle", visionPose.getRotation().getRadians());
            logData("Vision Pose Time", getLimelightTime());

            RobotPositionSender.addRobotPosition(new RobotState(
                    visionPose.getX(),
                    visionPose.getY(),
                    visionPose.getRotation().getRadians(),
                    getLimelightTime(),
                    "Vision Pose"
            ));


            Translation2d trackerTranslation = robotTracker.getLatencyCompedPoseMeters().getTranslation();

            logData("Tracker Translation X", trackerTranslation.getX());
            logData("Tracker Translation Y", trackerTranslation.getY());
            if (limelight.areCornersTouchingEdge()) {
                logData("Using Vision Info", "Corners touching edge");
            } else {
                if (dist2(robotTracker.getLatencyCompedPoseMeters().getTranslation(),
                        robotTranslation) < Constants.VISION_MANAGER_DISTANCE_THRESHOLD_SQUARED) {
                    if (DriverStation.isTeleopEnabled()) {
                        robotTracker.addVisionMeasurement(robotTranslation, getLimelightTime(), false);
                    }

                    robotPositionOffset = new Translation2d();
                    logData("Using Vision Info", "Using Vision Info");
                    loopsWithBadVision.set(0);
                    blinkinLED.setStatus(limelightUsingVisionStatus);
                    logData("IS VISION GOOD", true);
                } else {
                    if (loopsWithBadVision.incrementAndGet() > Constants.MAX_BAD_VISION_ITERATIONS) {
                        logData("IS VISION GOOD", false);
                    } else {
                        logData("IS VISION GOOD", true);
                    }
                    logData("Using Vision Info", "Position is too far from expected");
                    blinkinLED.setStatus(limelightTooFarFromExpectedStatus);
                }
            }
        } else {
            logData("Using Vision Info", "No target visible");
            blinkinLED.setStatus(limelightNotVisibleStatus);
        }
    }


    private static final ControllerDriveInputs CONTROLLER_DRIVE_NO_MOVEMENT = new ControllerDriveInputs(0, 0, 0);

    /**
     * For auto use only
     */
    @SuppressWarnings({"unused", "BusyWait"})
    public void shootBalls(double numBalls, Function<VisionManager, Boolean> additionalWait) throws InterruptedException {
        final @NotNull Drive drive = Drive.getInstance();
        final @NotNull Shooter shooter = Shooter.getInstance();

        forceVisionOn(this);

        do {
            if (drive.driveState == DriveState.RAMSETE) {
                drive.setAutoAiming(shootAndMove(CONTROLLER_DRIVE_NO_MOVEMENT, true, false));
            } else {
                autoTurnAndShoot(CONTROLLER_DRIVE_NO_MOVEMENT, true);
            }
            Thread.sleep(10); // Will exit if interrupted
        } while (shooter.getFeederWheelState() != FeederWheelState.FORWARD || !additionalWait.apply(this));

        double shootTime = numBalls * Constants.SHOOT_TIME_PER_BALL;
        double lastTime = Timer.getFPGATimestamp();
        double timeShooting = 0;

        while (timeShooting < shootTime) {
            if (drive.driveState == DriveState.RAMSETE) {
                drive.setAutoAiming(shootAndMove(CONTROLLER_DRIVE_NO_MOVEMENT, true, false));
            } else {
                autoTurnAndShoot(CONTROLLER_DRIVE_NO_MOVEMENT, true);
            }
            double time = Timer.getFPGATimestamp();
            if (shooter.getFeederWheelState() == FeederWheelState.FORWARD) {
                timeShooting += time - lastTime;
            }
            lastTime = time;
            Thread.sleep(10); // Will exit if interrupted
        }
        unForceVisionOn(this);
        shooter.setFiring(false);
        shooter.setSpeed(0);
        drive.setAutoAiming(false);
    }

    public void shootBalls(double numOfBalls) throws InterruptedException {
        shootBalls(numOfBalls, (visionManager -> true));
    }

    private boolean hasBeamBreakBroken = false;

    public void shootBallsUntilBeamBreakHasBroken(double numOfBalls) throws InterruptedException {
        hasBeamBreakBroken = false;
        shootBalls(numOfBalls, (visionManager -> {
            if (Hopper.getInstance().isBeamBroken()) hasBeamBreakBroken = true;
            return hasBeamBreakBroken;
        }));
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
                                                   Translation2d currentVelocity, Translation2d currentAcceleration) {
        return currentTranslation
                .plus(currentVelocity.times(predictAheadTime))
                .plus(currentAcceleration.times(0.5 * predictAheadTime * predictAheadTime));
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
    @NotNull Translation2d getVelocityAdjustedRelativeTranslation(
            @NotNull Translation2d relativeGoalTranslation, @NotNull Translation2d robotVelocity) {

        MutableTranslation2d fakeGoalPos = new MutableTranslation2d(relativeGoalTranslation);

        double relGoalX = relativeGoalTranslation.getX();
        double relGoalY = relativeGoalTranslation.getY();

        double velX = robotVelocity.getX();
        double velY = robotVelocity.getY();

        for (int i = 0; i < 20; i++) {
            //System.out.println("Iteration: " + i + " Fake Goal Pos: " + fakeGoalPos);
            double tof = getTimeOfFlight(fakeGoalPos);

            fakeGoalPos.set(
                    relGoalX + (velX * tof),
                    relGoalY + (velY * tof)
            );
        }
        return fakeGoalPos.getTranslation2d();
    }

    /**
     * @param translation2d The position of the target
     * @return the time of flight to the target
     */
    double getTimeOfFlight(Translation2d translation2d) {
        double distance = Units.metersToInches(translation2d.getNorm());

        double timeOfFlightFrames;
        if (distance < 113) {
            timeOfFlightFrames = ((0.03 / 30) * (distance - 113)) + (23.0 / 30);
        } else {
            timeOfFlightFrames = ((0.041 / 30) * (distance - 113)) + (23.0 / 30);
        }

        //timeOfFlightFrames = 0.000227991 * (distance * distance) - 0.0255545 * (distance) + 31.9542;
        return timeOfFlightFrames;
    }

    /**
     * @param predictAheadTime The amount of time ahead of the robot that the robot should predict
     * @return A translation that compensates for the robot's velocity. This is used to calculate the "fake" target position
     */
    @NotNull
    private Translation2d getAdjustedTranslation(double predictAheadTime) {
        Translation2d relativeRobotTranslation = getRelativeGoalTranslation();
        Translation2d robotVelocity = getRobotVel();
        return getVelocityAdjustedRelativeTranslation(
                predictFutureTranslation(predictAheadTime, relativeRobotTranslation, robotVelocity, getAccel()),
                robotVelocity.plus(getAccel().times(predictAheadTime))
        );
    }


    private static final Translation2d ZERO = new Translation2d();

    private Translation2d getAccel() {
        final @NotNull RobotTracker robotTracker = RobotTracker.getInstance();
        return Drive.getInstance().lastAcceleration;
        //return robotTracker.getAcceleration();
        //return ZERO;
    }

    private static final Rotation2d ROTATION_180 = new Rotation2d(Math.PI);

    /**
     * The front of our robot is the intake side, and we shoot backwards. Because of this we need to rotate our target angle by
     * 180 degrees.
     *
     * @return a rotation 180 degrees
     */
    private @NotNull Rotation2d shootToDriveRotation(@NotNull Rotation2d shootRotation) {
        return shootRotation.rotateBy(ROTATION_180);
    }

    public @NotNull Rotation2d aimPointToDriveRotation(@NotNull Translation2d translation2d) {
        return shootToDriveRotation(angleOf(translation2d));
    }
}
