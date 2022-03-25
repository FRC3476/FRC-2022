package frc.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.subsystem.BlinkinLED.BlinkinLedMode;
import frc.subsystem.BlinkinLED.LedStatus;
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
    private final @NotNull BlinkinLED blinkinLED = BlinkinLED.getInstance();

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
        logData("Is Robot Allowed Shoot Aiming",
                Math.abs((angleOf(getRelativeGoalTranslation())
                        .minus(robotTracker.getGyroAngle())).getRadians())
                        < getAllowedTurnError());
        logData("Is Robot Allowed Shoot Tilt",
                Math.abs(robotTracker.getGyro().getRoll()) < 3 && Math.abs(robotTracker.getGyro().getPitch()) < 3);

        Translation2d robotVelocity = getRobotVel();
        Translation2d relativeRobotTranslation = getRelativeGoalTranslation();
        logData("Relative Robot Translation X", relativeRobotTranslation.getX());
        logData("Relative Robot Translation Y", relativeRobotTranslation.getY());

        logData("Vision Robot Velocity X", robotVelocity.getX());
        logData("Vision Robot Velocity Y", robotVelocity.getY());

        Translation2d aimToPosition = getAdjustedTranslation(0.15);

        Translation2d fieldCentricCords =
                RobotTracker.getInstance().getLastEstimatedPoseMeters().getTranslation().minus(aimToPosition);
        logData("Calculated Target X", fieldCentricCords.getX());
        logData("Calculated Target Y", fieldCentricCords.getY());
    }


    public void shootAndMove(ControllerDriveInputs controllerDriveInputs, boolean useFieldRelative) {
        Translation2d aimToPosition = getAdjustedTranslation(0.15);

        double targetAngle = angleOf(aimToPosition).getRadians();

        // Get the angle that will be used in the future to calculate the end velocity of the turn
        Translation2d futureAimToPosition = getAdjustedTranslation(0.25);
        double futureTargetAngle = angleOf(futureAimToPosition).getRadians();

        drive.updateTurn(controllerDriveInputs,
                new State(targetAngle, (futureTargetAngle - targetAngle) * 10),
                useFieldRelative,
                0);

        updateShooterState(aimToPosition.getNorm());

        tryToShoot(aimToPosition, (futureTargetAngle - targetAngle) * 10, false);
    }


    public void autoTurnRobotToTarget(ControllerDriveInputs controllerDriveInputs, boolean fieldRelative) {
        drive.updateTurn(controllerDriveInputs, getAngleToTarget(), fieldRelative, getAllowedTurnError());

        updateShooterState(getDistanceToTarget());
        tryToShoot(getRelativeGoalTranslation(), 0, true);
    }

    private void tryToShoot(Translation2d aimToPosition, double targetAngularSpeed, boolean doSpeedCheck) {
        if (Math.abs((angleOf(aimToPosition).minus(robotTracker.getGyroAngle())).getRadians()) < getAllowedTurnError()
                && Math.abs(robotTracker.getLatencyCompedChassisSpeeds().omegaRadiansPerSecond - targetAngularSpeed)
                < Math.toRadians(8)
                && (drive.getSpeedSquared() < Constants.MAX_SHOOT_SPEED_SQUARED || !doSpeedCheck) &&
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
        logData("Shooter Distance to Target", Units.metersToInches(distanceToTarget));
        shooter.set(visionLookUpTable.getShooterPreset(Units.metersToInches(distanceToTarget)));
    }

    /**
     * @return the current robot velocity from the robot tracker
     */
    @Contract(pure = true)
    public Translation2d getRobotVel() {
        Rotation2d rotation2d = robotTracker.getGyroAngle();
        ChassisSpeeds chassisSpeeds = drive.getSwerveDriveKinematics().toChassisSpeeds(drive.getSwerveModuleStates());
        return new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond).rotateBy(rotation2d);
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
                .plus(Constants.GOAL_POSITION));
    }

    /**
     * @return current relative translation of the robot based on the robot tracker
     */
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
        Optional<Translation2d> visionTranslation = getVisionTranslation();
        visionTranslation.ifPresent(
                mutableTranslation2d -> {
                    robotTracker.addVisionMeasurement(
                            mutableTranslation2d,
                            getLimelightTime());
                    robotPositionOffset = new Translation2d();
                }
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
        double limelightTime = Timer.getFPGATimestamp(); //- (limelight.getLatency() / 1000.0) - (11.0 / 1000);
        logData("Limelight Latency", (limelight.getLatency() / 1000) + (11.0 / 1000));
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

    @Override
    public void update() {
        Pose2d robotTrackerPose = robotTracker.getLatencyCompedPoseMeters();
        Translation2d relativeGoalPos = getRelativeGoalTranslation();

        double angleToTarget = Math.atan2(relativeGoalPos.getY(), relativeGoalPos.getX());

        if (!limelight.isTargetVisible()) {
            blinkinLED.setStatus(limelightNotConnectedStatus);
        }

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
            logData("Vision Pose Angle", visionPose.getRotation().getRadians());
            logData("Vision Pose Time", getLimelightTime());

            if (MathUtil.dist2(robotTracker.getLatencyCompedPoseMeters().getTranslation().plus(robotPositionOffset),
                    robotTranslation) < Constants.VISION_MANAGER_DISTANCE_THRESHOLD_SQUARED
                    && !limelight.areCornersTouchingEdge()) {

                robotTracker.addVisionMeasurement(robotTranslation,
                        getLimelightTime());
                robotPositionOffset = new Translation2d();


                logData("Using Vision Info", "Using Vision Info");
                blinkinLED.setStatus(limelightUsingVisionStatus);
            } else {
                logData("Using Vision Info", "Position is too far from expected");
                blinkinLED.setStatus(limelightTooFarFromExpectedStatus);
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
                                                   Translation2d currentVelocity, Translation2d currentAcceleration) {
        return currentTranslation
                .plus(currentVelocity.times(predictAheadTime))
                .plus(currentTranslation.times(0.5 * predictAheadTime * predictAheadTime));
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

        Translation2d fakeGoalPos = relativeGoalTranslation;

        for (int i = 0; i < 40; i++) {
            //System.out.println("Iteration: " + i + " Fake Goal Pos: " + fakeGoalPos);
            double tof = getTimeOfFlight(fakeGoalPos);
            fakeGoalPos = relativeGoalTranslation.plus(robotVelocity.times(tof));
        }
        return fakeGoalPos;
    }

    /**
     * @param translation2d The position of the target
     * @return the time of flight to the target
     */
    double getTimeOfFlight(Translation2d translation2d) {
        double distance = Units.metersToInches(translation2d.getNorm());

        double timeOfFlightFrames;
        if (distance < 120) {
            timeOfFlightFrames = 28;
        } else {
            timeOfFlightFrames = (0.09 * (distance - 120)) + 28;
        }

        //timeOfFlightFrames = 0.000227991 * (distance * distance) - 0.0255545 * (distance) + 31.9542;
        return timeOfFlightFrames / 30;
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


    private Translation2d getAccel() {
        return robotTracker.getAcceleration();
    }
}
