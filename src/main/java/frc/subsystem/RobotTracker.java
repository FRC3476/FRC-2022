package frc.subsystem;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.jetbrains.annotations.NotNull;

public final class RobotTracker extends AbstractSubsystem {

    private final @NotNull AHRS gyroSensor;

    private static @NotNull RobotTracker instance = new RobotTracker();

    private final Drive drive = Drive.getInstance();

    public static @NotNull RobotTracker getInstance() {
        return RobotTracker.instance;
    }

    private Pose2d lastEstimatedPose = new Pose2d();

    private final @NotNull SwerveDriveOdometry swerveDriveOdometry;

    double lastTime = 0;

    private RobotTracker() {
        super(20);
        gyroSensor = new AHRS(SPI.Port.kMXP);
        // swerveDriveOdometry = new SwerveDrivePoseEstimator(
        //     drive.getGyroAngle(),
        //     new Pose2d(),
        //     drive.getSwerveDriveKinematics(),
        //     new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.02),
        //     new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.02),
        //     new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.1),
        //     20d/1000d);
        swerveDriveOdometry = new SwerveDriveOdometry(drive.getSwerveDriveKinematics(), gyroSensor.getRotation2d());
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
        return lastEstimatedPose.getRotation();
    }

    /**
     * Resets the position on the field to 0,0 with a rotation of 0 degrees
     */
    synchronized public void resetOdometry() {
        swerveDriveOdometry.resetPosition(new Pose2d(), gyroSensor.getRotation2d());
    }

    /**
     * Updates the robot's position on the field using forward kinematics and integration of the pose over time. This method
     * automatically calculates the current time to calculate period (difference between two timestamps). The period is used to
     * calculate the change in distance from a velocity. This also takes in an angle parameter which is used instead of the
     * angular rate that is calculated from forward kinematics.
     *
     */
    @Override
    public void update() {
        updateOdometry(WPIUtilJNI.now() * 1.0e-6, gyroSensor.getRotation2d(), drive.getSwerveModuleStates());
    }

    /**
     * /** Updates the robot's position on the field using forward kinematics and integration of the pose over time. This method
     * takes in the current time as a parameter to calculate period (difference between two timestamps). The period is used to
     * calculate the change in distance from a velocity. This also takes in an angle parameter which is used instead of the
     * angular rate that is calculated from forward kinematics.
     *
     * @param currentTimeSeconds The current time in seconds.
     * @param gyroAngle          The angle reported by the gyroscope.
     * @param moduleStates       The current state of all swerve modules. Please provide the states in the same order in which you
     *                           instantiated your SwerveDriveKinematics.
     */
    public void updateOdometry(double currentTimeSeconds, Rotation2d gyroAngle, SwerveModuleState[] moduleStates) {
        swerveDriveOdometry.updateWithTime(currentTimeSeconds, gyroAngle, moduleStates);
        synchronized (this) {
            lastEstimatedPose = swerveDriveOdometry.getPoseMeters();
        }
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
        swerveDriveOdometry.resetPosition(pose, gyroAngle);
    }

    /**
     * Returns the position of the robot on the field.
     *
     * @return The pose of the robot (x and y are in meters).
     */
    synchronized public Pose2d getPoseMeters() {
        return lastEstimatedPose;
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
}