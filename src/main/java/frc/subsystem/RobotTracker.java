package frc.subsystem;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.utility.Timer;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

import java.util.*;
import java.util.Map.Entry;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import static frc.robot.Constants.ROBOT_TRACKER_PERIOD;

public final class RobotTracker extends AbstractSubsystem {

    private final @NotNull AHRS gyroSensor;

    private static @NotNull RobotTracker instance = new RobotTracker();

    public static @NotNull RobotTracker getInstance() {
        return RobotTracker.instance;
    }

    private @NotNull Pose2d latestEstimatedPose = new Pose2d();
    private @NotNull ChassisSpeeds latestChassisSpeeds = new ChassisSpeeds();

    private @NotNull ChassisSpeeds latencyCompensatedChassisSpeeds = new ChassisSpeeds();

    private final SwerveDriveOdometry swerveDriveOdometry;
    private @NotNull Rotation2d gyroOffset = new Rotation2d();

    private double gyroRollVelocity = 0;
    private double gyroPitchVelocity = 0;
    private double lastGyroPitch = 0;
    private double lastGyroRoll = 0;

    private double maxGyroRoll = 0;
    private double minGyroRoll = 0;

    private final @NotNull SwerveDriveKinematics swerveDriveKinematics = Drive.getSwerveDriveKinematics();

    private final ReentrantReadWriteLock lock = new ReentrantReadWriteLock();

    private Translation2d acceleration = new Translation2d();


    private RobotTracker() {
        super(Constants.ROBOT_TRACKER_PERIOD, 5);
        gyroSensor = new AHRS(SPI.Port.kMXP, (byte) 200);
        gyroSensor.getRequestedUpdateRate();
        //@formatter:off
//        swerveDriveOdometry = new SwerveDrivePoseEstimator(
//                gyroSensor.getRotation2d(),
//                new Pose2d(),
//                drive.getSwerveDriveKinematics(),
//                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.02), // stateStdDevs – [x, y, theta]ᵀ, with units in meters and radians.
//                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.00035), // localMeasurementStdDevs – [theta], with units in radians.
//                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.2, 0.2, 0.07), // visionMeasurementStdDevs – [x, y, theta]ᵀ, with units in meters and radians.
//                (Constants.ROBOT_TRACKER_PERIOD * 2) / 1000.0d); //Only update every other tick

        swerveDriveOdometry = new SwerveDriveOdometry(
                swerveDriveKinematics,
                gyroSensor.getRotation2d(),
                new Pose2d()

        );
        //@formatter:on
        //swerveDriveOdometry = new SwerveDriveOdometry(drive.getSwerveDriveKinematics(), gyroSensor.getRotation2d());
    }

    /**
     * Add a vision measurement to the Unscented Kalman Filter. This will correct the odometry pose estimate while still
     * accounting for measurement noise.
     *
     * <p>This method can be called as infrequently as you want, as long as you are calling {@link
     * SwerveDrivePoseEstimator#update} every loop.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds      The timestamp of the vision measurement in seconds. Note that if you don't use your own time
     *                              source by calling {@link SwerveDrivePoseEstimator#updateWithTime} then you must use a
     *                              timestamp with an epoch since FPGA startup (i.e. the epoch of this timestamp is the same epoch
     *                              as Timer.getFPGATimestamp.) This means that you should use Timer.getFPGATimestamp as your time
     *                              source or sync the epochs.
     */
    public void addVisionMeasurement(Translation2d visionRobotPoseMeters, double timestampSeconds) {
        resetPosition(visionRobotPoseMeters);
//        try {
//            swerveDriveOdometry.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds); //Crashes the robot
//        } catch (RuntimeException e) {
//            kalmanFilterFailed();
//        }
    }

    public void calibrateGyro() {
        gyroSensor.calibrate();
    }

    public AHRS getGyro() {
        return gyroSensor;
    }


    /**
     * @return current rotation
     */
    public synchronized Rotation2d getAngle() {
        return latestEstimatedPose.getRotation();
    }

    private final List<Map.Entry<Double, Rotation2d>> previousGyroRotations = new ArrayList<>(102); // 1s of data at 10ms per

    double currentOdometryTime = -1;


    AtomicInteger failedTime = new AtomicInteger(0);

    private void kalmanFilterFailed() {
        logData("Kalman Filter Failed Counter", failedTime.incrementAndGet());
    }

    private boolean updateNextTick = true;

    /**
     * Updates the robot's position on the field using forward kinematics and integration of the pose over time. This method
     * automatically calculates the current time to calculate period (difference between two timestamps). The period is used to
     * calculate the change in distance from a velocity. This also takes in an angle parameter which is used instead of the
     * angular rate that is calculated from forward kinematics.
     */
    @Override
    public void update() {
        final Drive drive = Drive.getInstance();
        double time = WPIUtilJNI.now() * 1.0e-6; // seconds

        Rotation2d rawGyroSensor = gyroSensor.getRotation2d();
        synchronized (previousGyroRotations) {
            previousGyroRotations.add(Map.entry(time, gyroSensor.getRotation2d()));

            while (previousGyroRotations.size() > 100) {
                previousGyroRotations.remove(0);
            }
        }

        if (updateNextTick) {
            SwerveModuleState[] swerveModuleStates = drive.getSwerveModuleStates();
            updateOdometry(time, rawGyroSensor, swerveModuleStates);
            lock.writeLock().lock();
            try {
                latestEstimatedPose = swerveDriveOdometry.getPoseMeters();
                //gyroOffset = latestEstimatedPose.getRotation().minus(rawGyroSensor);

                latestChassisSpeeds = getRotatedSpeeds(
                        swerveDriveKinematics.toChassisSpeeds(swerveModuleStates),
                        getGyroAngle());

                latencyCompensatedChassisSpeeds = latestChassisSpeeds;
                currentOdometryTime = Timer.getFPGATimestamp();

                gyroPitchVelocity = (RobotTracker.getInstance().getGyro().getPitch() - lastGyroPitch)
                        / ((double) (ROBOT_TRACKER_PERIOD * 2) / 1000);
                gyroRollVelocity = (RobotTracker.getInstance().getGyro().getRoll() - lastGyroRoll)
                        / ((double) (ROBOT_TRACKER_PERIOD * 2) / 1000);

                lastGyroPitch = RobotTracker.getInstance().getGyro().getPitch();
                lastGyroRoll = RobotTracker.getInstance().getGyro().getRoll();


                acceleration = new Translation2d(getGyro().getWorldLinearAccelX(), getGyro().getWorldLinearAccelY())
                        .rotateBy(gyroSensor.getRotation2d().unaryMinus())
                        .rotateBy(getGyroAngle());

                if (maxGyroRoll < lastGyroRoll) {
                    maxGyroRoll = lastGyroRoll;
                }

                if (minGyroRoll > lastGyroRoll) {
                    minGyroRoll = lastGyroRoll;
                }
            } finally {
                lock.writeLock().unlock();
            }
        }


        updateNextTick = !updateNextTick;
        // Store sensor data for later. New data is always at the front of the list.
//        previousAbsolutePositions.add(0, Map.entry(time, drive.getWheelRotations()));
//        previousGyroRotations.add(0, Map.entry(time, gyroSensor.getRotation2d()));
//        previousAccelerometerData.add(0, Map.entry(time,
//                new Translation2d(gyroSensor.getWorldLinearAccelX(), gyroSensor.getWorldLinearAccelY()))); //Robot Centric
//
//        double currentOdometryTime = time - Constants.DRIVE_VELOCITY_MEASUREMENT_LATENCY;
//        if (previousAbsolutePositions.get(previousAbsolutePositions.size() - 1).getKey() > currentOdometryTime &&
//                previousGyroRotations.get(previousGyroRotations.size() - 1).getKey() > currentOdometryTime) {
//            double[] currentAbsolutePositions = getAbsolutePositions(currentOdometryTime);
//            Rotation2d currentGyroRotation = getGyroRotation(currentOdometryTime);
//            double[] swerveModuleSpeeds = drive.getModuleSpeeds();
//
//
//            SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
//            for (int i = 0; i < 4; i++) {
//                swerveModuleStates[i] = new SwerveModuleState(swerveModuleSpeeds[i],
//                        new Rotation2d(currentAbsolutePositions[i]));
//            }
//
//            updateOdometry(time, currentGyroRotation, swerveModuleStates);
//
//            for (Map.Entry<Double, Pose2d> visionUpdate : deferredVisionUpdates) {
//                if (visionUpdate.getKey() > currentOdometryTime) break;
//                swerveDriveOdometry.addVisionMeasurement(visionUpdate.getValue(), visionUpdate.getKey());
//            }
//
//            synchronized (deferredVisionUpdates) {
//                while (!deferredVisionUpdates.isEmpty() && deferredVisionUpdates.peek().getKey() <= currentOdometryTime) {
//                    var visionUpdate = deferredVisionUpdates.poll();
//                    if (visionUpdate.getKey() > currentOdometryTime) break;
//                    swerveDriveOdometry.addVisionMeasurement(visionUpdate.getValue(), visionUpdate.getKey());
//                    deferredVisionUpdates.remove(visionUpdate);
//                }
//                this.currentOdometryTime = currentOdometryTime;
//            }
//
//            //@formatter:off
//            Pose2d latestEstimatedPose = swerveDriveOdometry.getEstimatedPosition();
//
//            ChassisSpeeds latestChassisSpeeds = getRotatedSpeeds(drive.getSwerveDriveKinematics().toChassisSpeeds(swerveModuleStates),
//                    latestEstimatedPose.getRotation());
//            final MutableTranslation2d velocity = new MutableTranslation2d(latestChassisSpeeds.vxMetersPerSecond, latestChassisSpeeds.vyMetersPerSecond);
//            final MutableTranslation2d latencyCompensatedTranslation = new MutableTranslation2d(latestEstimatedPose.getTranslation());
//
//            Rotation2d gyroOffset = latestEstimatedPose.getRotation().minus(currentGyroRotation);
//
//            for (int i = previousAccelerometerData.size() - 1; i >= 0; i--) {
//                if (previousAccelerometerData.get(i).getKey() < currentOdometryTime) {
//                    previousAccelerometerData.remove(i);
//                } else {
//                    // Use accelerometer to calculate the current pose
//                    double dt;
//                    Translation2d acceleration;
//                    if (previousAccelerometerData.size() > i + 1) {
//                        dt = previousAccelerometerData.get(i).getKey() - previousAccelerometerData.get(i - 1).getKey();
//                        acceleration = previousAccelerometerData.get(i).getValue().plus(previousAccelerometerData.get(i - 1).getValue()).times(0.5)
//                                .rotateBy(previousGyroRotations.get(i).getValue().rotateBy(previousGyroRotations.get(i+1).getValue()).times(0.5));
//                    } else {
//                        dt = previousAccelerometerData.get(i).getKey() - currentOdometryTime;
//                        acceleration = previousAccelerometerData.get(i).getValue();
//                    }
//                    latencyCompensatedTranslation.plus(velocity.times(dt));
//                    velocity.plus(acceleration.times(dt));
//                    //@formatter:on
//                }
//            }
//
//            synchronized (this) {
//                this.gyroOffset = gyroOffset;
//                this.latestEstimatedPose = latestEstimatedPose;
//                this.latestChassisSpeeds = latestChassisSpeeds;
//                this.latencyCompensatedPose = new Pose2d(latencyCompensatedTranslation.getTranslation2d(),
//                        previousGyroRotations.get(0).getValue().rotateBy(gyroOffset));
//
//                this.latencyCompensatedChassisSpeeds = new ChassisSpeeds(velocity.getX(), velocity.getY(),
//                        (previousGyroRotations.get(0).getValue().getRadians()
//                                - previousGyroRotations.get(1).getValue().getRadians()) /
//                                (time - previousGyroRotations.get(1).getKey()));
//            }
//        }
    }

    /**
     * Converts a ChassisSpeeds to a field relative ChassisSpeeds.
     *
     * @return The field relative ChassisSpeeds.
     */
    @Contract(pure = true)
    private ChassisSpeeds getRotatedSpeeds(ChassisSpeeds speeds, Rotation2d rotation) {
        Translation2d velocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).rotateBy(rotation);
        return new ChassisSpeeds(velocity.getX(), velocity.getY(), speeds.omegaRadiansPerSecond);
    }

    private final Comparator<Map.Entry<Double, Rotation2d>> comparator = Comparator.comparingDouble(Entry::getKey);

    Rotation2d zero = new Rotation2d();

    /**
     * Will also delete the gyro measurements that are older than the current time.
     *
     * @param time the time of the measurement
     * @return the state of the gyro at the specified time
     */
    public Rotation2d getGyroRotation(double time) {
        synchronized (previousGyroRotations) {
            List<Map.Entry<Double, Rotation2d>> list = previousGyroRotations;

//        if (list.isEmpty()) {
//            System.out.println(list);
//            throw new IllegalStateException("Provided list is empty");
//        } else if (list.get(0).getKey() < time) {
//            System.out.println(list);
//            throw new IllegalArgumentException("Time is not in this list");
//        } else if (list.get(list.size() - 1).getKey() > time) {
//            System.out.println(list);
//            throw new IllegalArgumentException("Time is too far in the past");
//        }

            //int index = list.size() - ((int) ((Timer.getFPGATimestamp() - time) * 100));
            int index = Collections.binarySearch(list, Map.entry(time, zero), comparator);
            logData("Using index for rotation", index);
            if (index < 0) index = -index - 1;

            if (list.size() < 2) {
                return getGyroAngle();
            }
            if (index >= list.size()) {
                return list.get(list.size() - 1).getValue().plus(gyroOffset);
            }
            //if (index - 1 > list.size() || index < 0) return getGyroAngle();

            logData("Using Index Timer", list.get(index).getKey() - Timer.getFPGATimestamp());
            return list.get(index).getValue().plus(gyroOffset);
        }
    }


    /**
     * Updates the robot's position on the field using forward kinematics and integration of the pose over time. This method takes
     * in the current time as a parameter to calculate period (difference between two timestamps). The period is used to calculate
     * the change in distance from a velocity. This also takes in an angle parameter which is used instead of the angular rate
     * that is calculated from forward kinematics.
     *
     * @param currentTimeSeconds The current time in seconds.
     * @param gyroAngle          The angle reported by the gyroscope.
     * @param moduleStates       The current state of all swerve modules. Please provide the states in the same order in which you
     *                           instantiated your SwerveDriveKinematics.
     */
    private void updateOdometry(double currentTimeSeconds, Rotation2d gyroAngle, SwerveModuleState[] moduleStates) {
        swerveDriveOdometry.updateWithTime(currentTimeSeconds, gyroAngle, moduleStates);
    }

    /**
     * Resets the robot's position on the field. The gyroscope angle does not need to be reset here on the user's robot code. The
     * library automatically takes care of offsetting the gyro angle.
     *
     * @param pose The position on the field that your robot is at.
     */
    public void resetPosition(Pose2d pose) {
        resetPosition(pose, gyroSensor.getRotation2d());
    }

    public void resetPosition(Translation2d pose) {
        resetPosition(new Pose2d(pose, getGyroAngle()), gyroSensor.getRotation2d());
    }


    public void resetPosition(@NotNull Pose2d pose, @NotNull Rotation2d gyroAngle) {
        final Drive drive = Drive.getInstance();
        lock.writeLock().lock();
        try {

            gyroOffset = pose.getRotation().minus(gyroAngle);
            swerveDriveOdometry.resetPosition(pose, gyroAngle);
            latestEstimatedPose = pose;

            latestChassisSpeeds = getRotatedSpeeds(
                    Drive.getSwerveDriveKinematics().toChassisSpeeds(drive.getSwerveModuleStates()),
                    latestEstimatedPose.getRotation());

            latencyCompensatedChassisSpeeds = latestChassisSpeeds;
        } finally {
            lock.writeLock().unlock();
        }
    }

    /**
     * Returns the last estimated position of the robot on the field.
     * <p>
     * This method may have up to configured amount of latency. Use {@link RobotTracker#getLatencyCompedPoseMeters} if you need to
     * get the position at the current time.
     *
     * @return The pose of the robot (x and y are in meters).
     */
    @Contract(pure = true)
    public @NotNull Pose2d getLastEstimatedPoseMeters() {
        lock.readLock().lock();
        try {
            return latestEstimatedPose;
        } finally {
            lock.readLock().unlock();
        }
    }

    /**
     * Returns the position of the robot on the field.
     * <p>
     * This method returns the position of the robot at the current time. It integrates accelerometer data and gyroscope data to
     * get the current pose.
     *
     * @return The pose of the robot (x and y are in meters).
     */
    @Contract(pure = true)
    public @NotNull Pose2d getLatencyCompedPoseMeters() {
        return getLastEstimatedPoseMeters();
    }

    /**
     * Returns the last calculated velocity of the robot on the field.
     * <p>
     * This method may have up to configured amount of latency. Use {@link RobotTracker#getLatencyCompedChassisSpeeds} if you need
     * to get the velocity
     *
     * @return The velocity of the robot (x and y are in meters per second, Theta is in radians per second).
     */
    @Contract(pure = true)
    public @NotNull ChassisSpeeds getLastChassisSpeeds() {
        lock.readLock().lock();
        try {
            return latestChassisSpeeds;
        } finally {
            lock.readLock().unlock();
        }
    }

    /**
     * Returns the current velocity of the robot on the field.
     * <p>
     * This method gets uses data from the accelerometer and gyroscope to compensate for the tracking latency.
     *
     * @return The velocity of the robot (x and y are in meters per second, Theta is in radians per second).
     */
    @Contract(pure = true)
    public @NotNull ChassisSpeeds getLatencyCompedChassisSpeeds() {
        lock.readLock().lock();
        try {
            return latencyCompensatedChassisSpeeds;
        } finally {
            lock.readLock().unlock();
        }
    }


    @Override
    public void selfTest() {

    }

    @Override
    public void logData() {
        lock.readLock().lock();
        try {
            logData("Last Estimated Robot Pose X", getLastEstimatedPoseMeters().getX());
            logData("Last Estimated Robot Pose Y", getLastEstimatedPoseMeters().getY());
            logData("Last Estimated Robot Pose Angle", getLastEstimatedPoseMeters().getRotation().getRadians());
            logData("Gyro Robot Pose Angle", getGyroAngle().getRadians());
            logData("Last Estimated Robot Velocity X", getLastChassisSpeeds().vxMetersPerSecond);
            logData("Last Estimated Robot Velocity Y", getLastChassisSpeeds().vyMetersPerSecond);
            logData("Last Estimated Robot Velocity Theta", getLastChassisSpeeds().omegaRadiansPerSecond);

            logData("Gyro Pitch", RobotTracker.getInstance().getGyro().getPitch());
            logData("Gyro Pitch Velocity", gyroPitchVelocity);
            logData("Gyro Roll", RobotTracker.getInstance().getGyro().getRoll());
            logData("Gyro Roll Velocity", gyroRollVelocity);

            logData("Min Gyro Roll", minGyroRoll);
            logData("Max Gyro Roll", maxGyroRoll);


//        SmartDashboard.putNumber("Latency Comped Robot Pose X", getLatencyCompedPoseMeters().getX());
//        SmartDashboard.putNumber("Latency Comped Robot Pose Y", getLatencyCompedPoseMeters().getX());
//        SmartDashboard.putNumber("Latency Comped Robot Pose Angle", getLatencyCompedPoseMeters().getRotation().getDegrees());
//        SmartDashboard.putNumber("Latency Comped Robot Velocity X", getLatencyCompedChassisSpeeds().vxMetersPerSecond);
//        SmartDashboard.putNumber("Latency Comped Robot Velocity Y", getLatencyCompedChassisSpeeds().vyMetersPerSecond);
//        SmartDashboard.putNumber("Latency Comped Robot Velocity Theta", getLatencyCompedChassisSpeeds().omegaRadiansPerSecond);
        } finally {
            lock.readLock().unlock();
        }


        logData("Timestamp", currentOdometryTime);
    }

    @Override
    public void close() {
        gyroSensor.close();
        instance = new RobotTracker();
    }

    @Contract(pure = true)
    public double getGyroRollVelocity() {
        lock.readLock().lock();
        try {
            return gyroRollVelocity;
        } finally {
            lock.readLock().unlock();
        }
    }

    @Contract(pure = true)
    public double getGyroPitchVelocity() {
        lock.readLock().lock();
        try {
            return gyroPitchVelocity;
        } finally {
            lock.readLock().unlock();
        }
    }

    /**
     * @return The gyro angle offset so that it lines up with the robot tracker rotation.
     */
    @Contract(pure = true)
    public Rotation2d getGyroAngle() {
        lock.readLock().lock();
        try {
            return gyroSensor.getRotation2d().plus(gyroOffset);
        } finally {
            lock.readLock().unlock();
        }
    }

    public void resetGyro() {
        lock.writeLock().lock();
        try {
            getGyro().reset();
            swerveDriveOdometry.resetPosition(getLastEstimatedPoseMeters(), new Rotation2d(0));
        } finally {
            lock.writeLock().unlock();
        }
    }

    public Translation2d getAcceleration() {
        lock.readLock().lock();
        try {
            return acceleration;
        } finally {
            lock.readLock().unlock();
        }
    }
}