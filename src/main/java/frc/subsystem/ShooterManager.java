package frc.subsystem;

import com.dacubeking.AutoBuilder.robot.drawable.Circle;
import com.dacubeking.AutoBuilder.robot.drawable.Renderer;
import com.dacubeking.AutoBuilder.robot.utility.Vector2;
import com.google.common.collect.EvictingQueue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.subsystem.Drive.DriveState;
import frc.subsystem.Hopper.HopperState;
import frc.subsystem.Shooter.FeederWheelState;
import frc.utility.ControllerDriveInputs;
import frc.utility.Limelight;
import frc.utility.geometry.MutableTranslation2d;
import frc.utility.shooter.visionlookup.ShooterConfig;
import frc.utility.shooter.visionlookup.VisionLookUpTable;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Function;

import static frc.robot.Constants.*;
import static frc.utility.geometry.GeometryUtils.angleOf;

public final class ShooterManager extends AbstractSubsystem {

    public static final Circle[] EMPTY_CIRCLES_ARRAY = new Circle[0];
    public final @NotNull VisionLookUpTable visionLookUpTable = VisionLookUpTable.getInstance();
    @SuppressWarnings("UnstableApiUsage")
    private final @NotNull EvictingQueue<Circle> drawableShotPredictedLandingLocation = EvictingQueue.create(10);


    public void setShooterConfig(ShooterConfig shooterConfig) {
        visionLookUpTable.setShooterConfig(shooterConfig);
    }

    private static final Color8Bit LIGHT_BLUE = new Color8Bit(36, 191, 212);
    private static final Color8Bit RED = new Color8Bit(255, 0, 0);
    private static final Color8Bit GREEN = new Color8Bit(0, 255, 0);

    private ShooterManager() {
        super(Constants.SHOOTER_MANAGER_PERIOD, 1);
    }

    private static final ReentrantReadWriteLock SHOOTER_MANGER_INSTANCE_LOCK = new ReentrantReadWriteLock();
    private static @Nullable ShooterManager instance = null;

    public static @NotNull ShooterManager getInstance() {
        SHOOTER_MANGER_INSTANCE_LOCK.readLock().lock();
        try {
            if (instance != null) {
                return instance;
            }
        } finally {
            SHOOTER_MANGER_INSTANCE_LOCK.readLock().unlock();
        }

        SHOOTER_MANGER_INSTANCE_LOCK.writeLock().lock();
        try {
            return Objects.requireNonNullElseGet(instance, () -> instance = new ShooterManager());
        } finally {
            SHOOTER_MANGER_INSTANCE_LOCK.writeLock().unlock();
        }
    }

    @Override
    public void selfTest() {

    }

    double lastShotTime = 0;

    @Override
    @SuppressWarnings("UnstableApiUsage")
    public void logData() {
        final @NotNull RobotTracker robotTracker = RobotTracker.getInstance();
        final @NotNull Drive drive = Drive.getInstance();
        final @NotNull Shooter shooter = Shooter.getInstance();

        logData("Rotation Target", getAngleToTarget().getDegrees());

        logData("Allow Shooting Robot Speed", drive.getSpeedSquared() < Constants.MAX_SHOOT_SPEED_SQUARED);
        logData("Is Robot Allowed Shoot Tilt",
                Math.abs(robotTracker.getGyro().getRoll()) < 3 && Math.abs(robotTracker.getGyro().getPitch()) < 3);

        Translation2d robotVelocity = getRobotVel();
        Translation2d relativeRobotTranslation = getRelativeGoalTranslation();
        logData("Relative Robot Translation X", relativeRobotTranslation.getX());
        logData("Relative Robot Translation Y", relativeRobotTranslation.getY());

        logData("Shooter Manager Robot Velocity X", robotVelocity.getX());
        logData("Shooter Manager Robot Velocity Y", robotVelocity.getY());
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

        Circle shootPosition = new Circle((float) fieldCentricCords.getX(), (float) fieldCentricCords.getY(),
                (float) GOAL_RADIUS * 0.5f, LIGHT_BLUE);

        double tof = getTimeOfFlight(aimToPosition);

        Translation2d currentBallLandingPosition =
                new MutableTranslation2d(aimToPosition.getNorm(),
                        RobotTracker.getInstance().getGyroAngle().minus(ROTATION_OFFSET))
                        .minus(getRobotVel().getX() * tof, getRobotVel().getY() * tof);

        Circle currentDrawableShotPredictedLandingLocation = new Circle(
                new Vector2((float) currentBallLandingPosition.getX(), (float) currentBallLandingPosition.getY()),
                0.1f, GREEN
        );

        if (lastShotTime < shooter.getLastShotTime()) {
            lastShotTime = shooter.getLastShotTime();
            drawableShotPredictedLandingLocation.add(new Circle(
                    new Vector2((float) currentBallLandingPosition.getX(), (float) currentBallLandingPosition.getY()),
                    0.1f, RED
            ));
        }

        // This feels awful, is there a better way to do this?
        ArrayList<Circle> circles = new ArrayList<>();
        circles.add(shootPosition);
        circles.add(currentDrawableShotPredictedLandingLocation);
        circles.addAll(drawableShotPredictedLandingLocation);
        Renderer.render(circles.toArray(EMPTY_CIRCLES_ARRAY));


        double allowedTurnError = getAllowedTurnError(aimToPosition.getNorm());

        logData("Allowed Turn Error", allowedTurnError);
    }

    public void shootAndMove(ControllerDriveInputs controllerDriveInputs, boolean useFieldRelative) {
        shootAndMove(controllerDriveInputs, useFieldRelative, true);
    }

    private Rotation2d ROTATION_OFFSET = Rotation2d.fromDegrees(1.5);

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

        Optional<Translation2d> visionTranslation = VisionManager.getInstance().getVisionTranslation();
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
        final @NotNull VisionManager visionManager = VisionManager.getInstance();


        double turnError = aimPointToDriveRotation(aimPoint)
                .plus(ROTATION_OFFSET).minus(robotTracker.getGyroAngle()).getRadians();
        final boolean isAimed = Math.abs(turnError) < getAllowedTurnError(aimPoint.getNorm());

        final boolean isTurningSpeedCorrect =
                Math.abs(robotTracker.getLatencyCompedChassisSpeeds().omegaRadiansPerSecond - targetAngularSpeed)
                        < Math.toRadians(8);

        final boolean isUnderAccelLimit = getAccel().getNorm() < MAX_ACCELERATION_WHILE_SHOOTING;
        final boolean isStopped = (drive.getSpeedSquared() < Constants.MAX_SHOOT_SPEED_SQUARED || !doSpeedCheck);
        final boolean isFlatOnGround = (Math.abs(robotTracker.getGyro().getRoll()) < 6 &&
                Math.abs(robotTracker.getGyro().getPitch()) < 6) || IS_PRACTICE; //The roborio on the practice bot is tilted a
        // bit and cause this check to fail

        logData("Is allowed Shoot Turn Speed", isTurningSpeedCorrect);
        logData("Is Robot Allowed Shoot Aiming", isAimed);
        logData("Is Robot Allowed Shoot Acceleration", isUnderAccelLimit);
        logData("Is Robot Allowed Shoot Stopped",  isStopped);
        logData("Is Robot Allowed Shoot On Flat Ground", isFlatOnGround);

        final boolean hasBadVision = visionManager.getLoopsWithBadVision() < Constants.MAX_BAD_VISION_ITERATIONS;

        Translation2d robotPosition = RobotTracker.getInstance().getLastEstimatedPoseMeters().getTranslation();
        if (isAimed && isTurningSpeedCorrect && isUnderAccelLimit && isStopped && isFlatOnGround && hasBadVision) {
            shooter.setFiring(true);
            if (shooter.isFiring()) {
                double time = Timer.getFPGATimestamp();
                if (!checksPassedLastTime && lastPrintTime + 1 < time) {
                    lastPrintTime = time;
                    checksPassedLastTime = true;
                    System.out.println(
                            "Shooting at " + (150 - DriverStation.getMatchTime()) + " Distance:  "
                                    + Units.metersToInches(aimPoint.getNorm()) + " "
                                    + "Accel: " + getAccel().getNorm() +
                                    " RT Angle To Target: " + turnError
                                    + " LL Angle to Target: " + Limelight.getInstance().getHorizontalOffset()
                                    + " Shooter Wheel Speed: " + shooter.getShooterRPM()
                                    + " Position: X: " + robotPosition.getX() + " Y: " + robotPosition.getY());
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
     * @return field relative translation with the origin at the goal
     */
    Translation2d getRelativeGoalTranslation() {
        final @NotNull RobotTracker robotTracker = RobotTracker.getInstance();
        return robotTracker.getLatencyCompedPoseMeters().getTranslation()
                .minus(GOAL_POSITION);
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

    /**
     * {@code Math.tan(Constants.GOAL_RADIUS / getDistanceToTarget())}
     *
     * @return The allowed turn error in radians
     */
    private double getAllowedTurnError() {
        return getAllowedTurnError(getDistanceToTarget());
    }

    /**
     * {@code Math.abs(Math.atan2((Constants.GOAL_RADIUS * 0.5), distance));}
     *
     * @return The allowed turn error in radians
     */
    private double getAllowedTurnError(double distance) {
        return Math.abs(Math.atan2((Constants.GOAL_RADIUS * 0.5), distance));
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


    private static final ControllerDriveInputs CONTROLLER_DRIVE_NO_MOVEMENT = new ControllerDriveInputs(0, 0, 0);

    /**
     * For auto use only
     */
    @SuppressWarnings({"unused", "BusyWait"})
    public void shootBalls(double shootTime, Function<ShooterManager, Boolean> additionalWait) throws InterruptedException {
        final @NotNull Drive drive = Drive.getInstance();
        final @NotNull Shooter shooter = Shooter.getInstance();

        VisionManager.getInstance().forceVisionOn(this);

        do {
            if (drive.driveState == DriveState.RAMSETE) {
                drive.setAutoAiming(shootAndMove(CONTROLLER_DRIVE_NO_MOVEMENT, true, false));
            } else {
                autoTurnAndShoot(CONTROLLER_DRIVE_NO_MOVEMENT, true);
            }
            Thread.sleep(10); // Will exit if interrupted
        } while (shooter.getFeederWheelState() != FeederWheelState.FORWARD || !additionalWait.apply(this));

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
        VisionManager.getInstance().unForceVisionOn(this);
        shooter.setFiring(false);
        shooter.setSpeed(0);
        drive.setAutoAiming(false);
    }

    public void shootBalls(double shootTime) throws InterruptedException {
        shootBalls(shootTime, (visionManager -> true));
    }

    private boolean hasBeamBreakBroken = false;

    public void shootBallsUntilBeamBreakHasBroken(double shootTime) throws InterruptedException {
        hasBeamBreakBroken = false;
        shootBalls(shootTime, (visionManager -> {
            if (Hopper.getInstance().isBeamBroken()) hasBeamBreakBroken = true;
            return hasBeamBreakBroken;
        }));
    }

    public void update() {
        Translation2d robotTranslation = RobotTracker.getInstance().getLastEstimatedPoseMeters().getTranslation();

//        if (robotTranslation.getX() < 7.4 ) {
//            ROTATION_OFFSET = Rotation2d.fromDegrees(1.5);
//        } else if (robotTranslation.getY() > 0) {
//            ROTATION_OFFSET = Rotation2d.fromDegrees(0.75);
//        } else {
//            ROTATION_OFFSET = Rotation2d.fromDegrees(1.1);
//        }
    }

    @Override
    public void close() throws Exception {

    }
}
