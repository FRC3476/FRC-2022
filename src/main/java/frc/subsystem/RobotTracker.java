package frc.subsystem;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.jetbrains.annotations.NotNull;

import java.util.*;

public final class RobotTracker extends AbstractSubsystem {

    private final @NotNull AHRS gyroSensor;

    private static @NotNull RobotTracker instance = new RobotTracker();

    private final Drive drive = Drive.getInstance();
    ;

    public static @NotNull RobotTracker getInstance() {
        return RobotTracker.instance;
    }

    private @NotNull ChassisSpeeds lastChassisSpeeds = new ChassisSpeeds();
    private @NotNull Pose2d latestEstimatedPose = new Pose2d();
    private @NotNull ChassisSpeeds latestChassisSpeeds = new ChassisSpeeds();

    private @NotNull Pose2d latencyCompensatedPose = new Pose2d();
    private @NotNull ChassisSpeeds latencyCompensatedChassisSpeeds = new ChassisSpeeds();

    private final @NotNull SwerveDrivePoseEstimator swerveDriveOdometry;
    double lastTime = 0;
    private @NotNull Rotation2d gyroOffset = new Rotation2d();


    private RobotTracker() {
        super(10);
        gyroSensor = new AHRS(SPI.Port.kMXP, (byte) 100);
        gyroSensor.getRequestedUpdateRate();
        swerveDriveOdometry = new SwerveDrivePoseEstimator(
                gyroSensor.getRotation2d(),
                new Pose2d(),
                drive.getSwerveDriveKinematics(),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.02),
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.00035),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.2, 0.2, 0.07),
                20.0d / 1000.0d);
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
    public synchronized void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        if (timestampSeconds > currentOdometryTime) {
            swerveDriveOdometry.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
        } else {
            deferredVisionUpdates.add(Map.entry(timestampSeconds, visionRobotPoseMeters));
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

    double currentOdometryTime = -1;

    /**
     * Updates the robot's position on the field using forward kinematics and integration of the pose over time. This method
     * automatically calculates the current time to calculate period (difference between two timestamps). The period is used to
     * calculate the change in distance from a velocity. This also takes in an angle parameter which is used instead of the
     * angular rate that is calculated from forward kinematics.
     */
    @Override
    public void update() {
        double time = WPIUtilJNI.now() * 1.0e-6;

        previousAbsolutePositions.add(Map.entry(time, drive.getAbsolutePositions()));
        previousGyroRotations.add(Map.entry(time, gyroSensor.getRotation2d()));


        double currentOdometryTime = time - 0.112;
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

            synchronized (this) {
                this.currentOdometryTime = time;

                while (!deferredVisionUpdates.isEmpty() && deferredVisionUpdates.peek().getKey() <= currentOdometryTime) {
                    var visionUpdate = deferredVisionUpdates.poll();
                    if (visionUpdate.getKey() > currentOdometryTime) break;
                    swerveDriveOdometry.addVisionMeasurement(visionUpdate.getValue(), visionUpdate.getKey());
                    deferredVisionUpdates.remove(visionUpdate);
                }


                latestEstimatedPose = swerveDriveOdometry.getEstimatedPosition();
                gyroOffset = latestEstimatedPose.getRotation().minus(currentGyroRotation);
            }
        }
    }

    private final Comparator comparator = (o1, o2) -> Double.compare(((Map.Entry<Double, ?>) o1).getKey(), (double) o2);


    /**
     * Will also delete the vision measurements that are older than the current time.
     *
     * @param time the time of the measurement
     * @return the state of the absolute encoders at the specified time
     */
    public double[] getAbsolutePositions(double time) {
        if (previousAbsolutePositions.isEmpty()) {
            throw new IllegalStateException("No previous positions");
        } else if (previousAbsolutePositions.get(0).getKey() < time) {
            throw new IllegalArgumentException("Time is not in this list");
        } else if (previousAbsolutePositions.get(previousAbsolutePositions.size() - 1).getKey() > time) {
            throw new IllegalArgumentException("Time is too far in the past");
        }

        int index = Collections.binarySearch(previousAbsolutePositions, time, comparator);
        if (index < 0) index = -index - 1;

        if (previousAbsolutePositions.size() > index + 2) {
            previousAbsolutePositions.subList(index + 2, previousAbsolutePositions.size()).clear();
        }

        return previousAbsolutePositions.get(index).getValue();
    }

    /**
     * Will also delete the gyro measurements that are older than the current time.
     *
     * @param time the time of the measurement
     * @return the state of the gyro at the specified time
     */
    public Rotation2d getGyroRotation(double time) {
        if (previousGyroRotations.isEmpty()) {
            throw new IllegalStateException("No previous rotations");
        } else if (previousGyroRotations.get(0).getKey() < time) {
            throw new IllegalArgumentException("Time is not in this list");
        } else if (previousGyroRotations.get(previousGyroRotations.size() - 1).getKey() > time) {
            throw new IllegalArgumentException("Time is too far in the past");
        }

        int index = Collections.binarySearch(previousGyroRotations, time, comparator);
        if (index < 0) index = -index - 1;

        if (previousGyroRotations.size() > index + 2) {
            previousGyroRotations.subList(index + 2, previousGyroRotations.size()).clear();
        }

        return previousGyroRotations.get(index).getValue();
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

    synchronized public void resetPosition(Pose2d pose, Rotation2d gyroAngle) {
        gyroOffset = gyroAngle.unaryMinus().rotateBy(pose.getRotation());
        swerveDriveOdometry.resetPosition(pose, gyroAngle);
    }

    /**
     * Returns the position of the robot on the field.
     *
     * @return The pose of the robot (x and y are in meters).
     */
    synchronized public Pose2d getPoseMeters() {
        return latestEstimatedPose;
    }

    @Override
    public void selfTest() {

    }

    @Override
    public void logData() {
        SmartDashboard.putNumber("Robot Pose X", getPoseMeters().getX());
        SmartDashboard.putNumber("Robot Pose Y", getPoseMeters().getX());
        SmartDashboard.putNumber("Robot Pose Angle", getPoseMeters().getRotation().getDegrees());
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
        swerveDriveOdometry.resetPosition(getPoseMeters(), new Rotation2d(0));
    }
}