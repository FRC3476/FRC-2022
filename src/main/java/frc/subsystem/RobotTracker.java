package frc.subsystem;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.utility.geometry.MutableTranslation2d;
import org.jetbrains.annotations.NotNull;

import java.util.*;

@SuppressWarnings("unused")
public final class RobotTracker extends AbstractSubsystem {

    private final @NotNull AHRS gyroSensor;

    private static @NotNull RobotTracker instance = new RobotTracker();

    private final Drive drive = Drive.getInstance();

    public static @NotNull RobotTracker getInstance() {
        return RobotTracker.instance;
    }

    private @NotNull Pose2d latestEstimatedPose = new Pose2d();
    private @NotNull ChassisSpeeds latestChassisSpeeds = new ChassisSpeeds();

    private @NotNull Pose2d latencyCompensatedPose = new Pose2d();
    private @NotNull ChassisSpeeds latencyCompensatedChassisSpeeds = new ChassisSpeeds();

    private final @NotNull SwerveDrivePoseEstimator swerveDriveOdometry;
    private @NotNull Rotation2d gyroOffset = new Rotation2d();


    private RobotTracker() {
        super(Constants.ROBOT_TRACKER_PERIOD);
        gyroSensor = new AHRS(SPI.Port.kMXP, (byte) 100);
        gyroSensor.getRequestedUpdateRate();
        //@formatter:off
        swerveDriveOdometry = new SwerveDrivePoseEstimator(
                gyroSensor.getRotation2d(),
                new Pose2d(),
                drive.getSwerveDriveKinematics(),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.02), // stateStdDevs – [x, y, theta]ᵀ, with units in meters and radians.
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.00035), // localMeasurementStdDevs – [theta], with units in radians.
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.2, 0.2, 0.07), // visionMeasurementStdDevs – [x, y, theta]ᵀ, with units in meters and radians.
                Constants.ROBOT_TRACKER_PERIOD / 1000.0d);
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
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        synchronized (deferredVisionUpdates) {
            if (timestampSeconds > currentOdometryTime) {
                swerveDriveOdometry.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
            } else {
                deferredVisionUpdates.add(Map.entry(timestampSeconds, visionRobotPoseMeters));
            }
        }
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

    /**
     * Resets the position on the field to 0,0 with a rotation of 0 degrees
     */
    synchronized public void resetOdometry() {
        swerveDriveOdometry.resetPosition(new Pose2d(), gyroSensor.getRotation2d());
    }

    private final List<Map.Entry<Double, double[]>> previousAbsolutePositions = new ArrayList<>(20);
    private final List<Map.Entry<Double, Rotation2d>> previousGyroRotations = new ArrayList<>(20);
    private final Queue<Map.Entry<Double, Pose2d>> deferredVisionUpdates = new ArrayDeque<>(20);
    List<Map.Entry<Double, Translation2d>> previousAccelerometerData = new ArrayList<>(20);

    double currentOdometryTime = -1;

    /**
     * Updates the robot's position on the field using forward kinematics and integration of the pose over time. This method
     * automatically calculates the current time to calculate period (difference between two timestamps). The period is used to
     * calculate the change in distance from a velocity. This also takes in an angle parameter which is used instead of the
     * angular rate that is calculated from forward kinematics.
     */
    @Override
    public void update() {
        double time = WPIUtilJNI.now() * 1.0e-6; // seconds

        // Store sensor data for later. New data is always at the front of the list.
        previousAbsolutePositions.add(0, Map.entry(time, drive.getWheelRotations()));
        previousGyroRotations.add(0, Map.entry(time, gyroSensor.getRotation2d()));
        previousAccelerometerData.add(0, Map.entry(time,
                new Translation2d(gyroSensor.getWorldLinearAccelX(), gyroSensor.getWorldLinearAccelY()))); //Robot Centric

        double currentOdometryTime = time - Constants.DRIVE_VELOCITY_MEASUREMENT_LATENCY;
        if (previousAbsolutePositions.get(previousAbsolutePositions.size() - 1).getKey() > currentOdometryTime &&
                previousGyroRotations.get(previousGyroRotations.size() - 1).getKey() > currentOdometryTime) {

            double[] currentAbsolutePositions = getAbsolutePositions(currentOdometryTime);
            Rotation2d currentGyroRotation = getGyroRotation(currentOdometryTime);
            double[] swerveModuleSpeeds = drive.getModuleSpeeds();


            SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
            for (int i = 0; i < 4; i++) {
                swerveModuleStates[i] = new SwerveModuleState(swerveModuleSpeeds[i],
                        new Rotation2d(currentAbsolutePositions[i]));
            }

            updateOdometry(time, currentGyroRotation, swerveModuleStates);

            for (Map.Entry<Double, Pose2d> visionUpdate : deferredVisionUpdates) {
                if (visionUpdate.getKey() > currentOdometryTime) break;
                swerveDriveOdometry.addVisionMeasurement(visionUpdate.getValue(), visionUpdate.getKey());
            }

            synchronized (deferredVisionUpdates) {
                while (!deferredVisionUpdates.isEmpty() && deferredVisionUpdates.peek().getKey() <= currentOdometryTime) {
                    var visionUpdate = deferredVisionUpdates.poll();
                    if (visionUpdate.getKey() > currentOdometryTime) break;
                    swerveDriveOdometry.addVisionMeasurement(visionUpdate.getValue(), visionUpdate.getKey());
                    deferredVisionUpdates.remove(visionUpdate);
                }
                this.currentOdometryTime = currentOdometryTime;
            }

            //@formatter:off
            Pose2d latestEstimatedPose = swerveDriveOdometry.getEstimatedPosition();

            ChassisSpeeds latestChassisSpeeds = getRotatedSpeeds(drive.getSwerveDriveKinematics().toChassisSpeeds(swerveModuleStates),
                    latestEstimatedPose.getRotation());
            final MutableTranslation2d velocity = new MutableTranslation2d(latestChassisSpeeds.vxMetersPerSecond, latestChassisSpeeds.vyMetersPerSecond);
            final MutableTranslation2d latencyCompensatedTranslation = new MutableTranslation2d(latestEstimatedPose.getTranslation());

            Rotation2d gyroOffset = latestEstimatedPose.getRotation().minus(currentGyroRotation);

            for (int i = previousAccelerometerData.size() - 1; i >= 0; i--) {
                if (previousAccelerometerData.get(i).getKey() < currentOdometryTime) {
                    previousAccelerometerData.remove(i);
                } else {
                    // Use accelerometer to calculate the current pose
                    double dt;
                    Translation2d acceleration;
                    if (previousAccelerometerData.size() > i + 1) {
                        dt = previousAccelerometerData.get(i).getKey() - previousAccelerometerData.get(i - 1).getKey();
                        acceleration = previousAccelerometerData.get(i).getValue().plus(previousAccelerometerData.get(i - 1).getValue()).times(0.5)
                                .rotateBy(previousGyroRotations.get(i).getValue().rotateBy(previousGyroRotations.get(i+1).getValue()).times(0.5));
                    } else {
                        dt = previousAccelerometerData.get(i).getKey() - currentOdometryTime;
                        acceleration = previousAccelerometerData.get(i).getValue();
                    }
                    latencyCompensatedTranslation.plus(velocity.times(dt));
                    velocity.plus(acceleration.times(dt));
                    //@formatter:on
                }
            }

            synchronized (this) {
                this.gyroOffset = gyroOffset;
                this.latestEstimatedPose = latestEstimatedPose;
                this.latestChassisSpeeds = latestChassisSpeeds;
                this.latencyCompensatedPose = new Pose2d(latencyCompensatedTranslation.getTranslation2d(),
                        previousGyroRotations.get(0).getValue().rotateBy(gyroOffset));

                this.latencyCompensatedChassisSpeeds = new ChassisSpeeds(velocity.getX(), velocity.getY(),
                        (previousGyroRotations.get(0).getValue().getRadians()
                                - previousGyroRotations.get(1).getValue().getRadians()) /
                                (time - previousGyroRotations.get(1).getKey()));
            }
        }
    }

    /**
     * Converts a ChassisSpeeds to a field relative ChassisSpeeds.
     *
     * @return The field relative ChassisSpeeds.
     */
    private ChassisSpeeds getRotatedSpeeds(ChassisSpeeds speeds, Rotation2d rotation) {
        double dist = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        return new ChassisSpeeds(
                dist * rotation.getCos(),
                dist * rotation.getSin(),
                speeds.omegaRadiansPerSecond
        );
    }

    private final Comparator comparator = (o1, o2) -> Double.compare(((Map.Entry<Double, ?>) o1).getKey(), (double) o2);

    /**
     * Will also delete the vision measurements that are older than the current time.
     *
     * @param time the time of the measurement
     * @return the state of the absolute encoders at the specified time
     */
    private double[] getAbsolutePositions(double time) {
        return getPositionOnListForTime(previousAbsolutePositions, time);
    }

    /**
     * Will also delete the gyro measurements that are older than the current time.
     *
     * @param time the time of the measurement
     * @return the state of the gyro at the specified time
     */
    private Rotation2d getGyroRotation(double time) {
        return getPositionOnListForTime(previousGyroRotations, time);
    }

    /**
     * Will also delete entries on the list that are past the specified time.
     *
     * @param list the list to search
     * @param time the time to search for
     * @return the index of the value
     */
    private <T> T getPositionOnListForTime(List<Map.Entry<Double, T>> list, double time) {
        if (list.isEmpty()) {
            throw new IllegalStateException("Provided list is empty");
        } else if (list.get(0).getKey() < time) {
            throw new IllegalArgumentException("Time is not in this list");
        } else if (list.get(list.size() - 1).getKey() > time) {
            throw new IllegalArgumentException("Time is too far in the past");
        }

        int index = Collections.binarySearch(list, time, comparator);
        if (index < 0) index = -index - 1;

        // Remove all entries that are past the time
        if (list.size() > index + 2) {
            list.subList(index + 2, list.size()).clear();
        }

        return list.get(index).getValue();
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
    public void updateOdometry(double currentTimeSeconds, Rotation2d gyroAngle, SwerveModuleState[] moduleStates) {
        swerveDriveOdometry.updateWithTime(currentTimeSeconds, gyroAngle, moduleStates);
    }

    /**
     * Resets the robot's position on the field. The gyroscope angle does not need to be reset here on the user's robot code. The
     * library automatically takes care of offsetting the gyro angle.
     *
     * @param pose The position on the field that your robot is at.
     */
    synchronized public void resetPosition(Pose2d pose) {
        resetPosition(pose, gyroSensor.getRotation2d());
    }

    synchronized public void resetPosition(@NotNull Pose2d pose, @NotNull Rotation2d gyroAngle) {
        gyroOffset = gyroAngle.unaryMinus().rotateBy(pose.getRotation());
        swerveDriveOdometry.resetPosition(pose, gyroAngle);
    }

    /**
     * Returns the last estimated position of the robot on the field.
     * <p>
     * This method may have up to configured amount of latency. Use {@link RobotTracker#getLatencyCompedPoseMeters} if you need to
     * get the position at the current time.
     *
     * @return The pose of the robot (x and y are in meters).
     */
    public synchronized @NotNull Pose2d getLastEstimatedPoseMeters() {
        return latestEstimatedPose;
    }

    /**
     * Returns the position of the robot on the field.
     * <p>
     * This method returns the position of the robot at the current time. It integrates accelerometer data and gyroscope data to
     * get the current pose.
     *
     * @return The pose of the robot (x and y are in meters).
     */
    public synchronized @NotNull Pose2d getLatencyCompedPoseMeters() {
        return latencyCompensatedPose;
    }

    /**
     * Returns the last calculated velocity of the robot on the field.
     * <p>
     * This method may have up to configured amount of latency. Use {@link RobotTracker#getLatencyCompedChassisSpeeds} if you need
     * to get the velocity
     *
     * @return The velocity of the robot (x and y are in meters per second, Theta is in radians per second).
     */
    public synchronized @NotNull ChassisSpeeds getLastChassisSpeeds() {
        return latestChassisSpeeds;
    }

    /**
     * Returns the current velocity of the robot on the field.
     * <p>
     * This method gets uses data from the accelerometer and gyroscope to compensate for the tracking latency.
     *
     * @return The velocity of the robot (x and y are in meters per second, Theta is in radians per second).
     */
    public synchronized @NotNull ChassisSpeeds getLatencyCompedChassisSpeeds() {
        return latencyCompensatedChassisSpeeds;
    }


    @Override
    public void selfTest() {

    }

    @Override
    public void logData() {
        SmartDashboard.putNumber("Last Estimated Robot Pose X", getLastEstimatedPoseMeters().getX());
        SmartDashboard.putNumber("Last Estimated Robot Pose Y", getLastEstimatedPoseMeters().getX());
        SmartDashboard.putNumber("Last Estimated Robot Pose Angle", getLastEstimatedPoseMeters().getRotation().getDegrees());
        SmartDashboard.putNumber("Last Estimated Robot Velocity X", getLastChassisSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("Last Estimated Robot Velocity Y", getLastChassisSpeeds().vyMetersPerSecond);
        SmartDashboard.putNumber("Last Estimated Robot Velocity Theta", getLastChassisSpeeds().omegaRadiansPerSecond);

        SmartDashboard.putNumber("Latency Comped Robot Pose X", getLatencyCompedPoseMeters().getX());
        SmartDashboard.putNumber("Latency Comped Robot Pose Y", getLatencyCompedPoseMeters().getX());
        SmartDashboard.putNumber("Latency Comped Robot Pose Angle", getLatencyCompedPoseMeters().getRotation().getDegrees());
        SmartDashboard.putNumber("Latency Comped Robot Velocity X", getLatencyCompedChassisSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("Latency Comped Robot Velocity Y", getLatencyCompedChassisSpeeds().vyMetersPerSecond);
        SmartDashboard.putNumber("Latency Comped Robot Velocity Theta", getLatencyCompedChassisSpeeds().omegaRadiansPerSecond);

        SmartDashboard.putNumber("Timestamp", currentOdometryTime);
    }

    @Override
    public void close() {
        gyroSensor.close();
        instance = new RobotTracker();
    }

    /**
     * @return The gyro angle offset so that it lines up with the robot tracker rotation.
     */
    public synchronized Rotation2d getGyroAngle() {
        return gyroSensor.getRotation2d().rotateBy(gyroOffset);
    }

    public void resetGyro() {
        getGyro().reset();
        swerveDriveOdometry.resetPosition(getLastEstimatedPoseMeters(), new Rotation2d(0));
    }
}