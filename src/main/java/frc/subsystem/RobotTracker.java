package frc.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class RobotTracker extends AbstractSubsystem {

    private static RobotTracker instance = new RobotTracker();

    private final Drive drive = Drive.getInstance();

    public static RobotTracker getInstance() {
        return RobotTracker.instance;
    }

    private Pose2d lastEstimatedPose = new Pose2d();

    private final SwerveDriveOdometry swerveDriveOdometry;

    private RobotTracker() {
        super(20);
        // swerveDriveOdometry = new SwerveDrivePoseEstimator(
        //     drive.getGyroAngle(),
        //     new Pose2d(),
        //     drive.getSwerveDriveKinematics(),
        //     new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.02),
        //     new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.02),
        //     new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.1),
        //     20d/1000d);
        swerveDriveOdometry = new SwerveDriveOdometry(drive.getSwerveDriveKinematics(), drive.getGyroAngle());
    }


    /**
     * @return current rotation
     */
    public synchronized Rotation2d getGyroAngle() {
        return lastEstimatedPose.getRotation();

    }

    /**
     * Resets the position on the field to 0,0 with a rotation of 0 degrees
     */
    synchronized public void resetOdometry() {
        swerveDriveOdometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(drive.getAngle()));
    }

    /**
     * Updates the robot's position on the field using forward kinematics and integration of the pose over time. This method
     * automatically calculates the current time to calculate period (difference between two timestamps). The period is used to
     * calculate the change in distance from a velocity. This also takes in an angle parameter which is used instead of the
     * angular rate that is calculated from forward kinematics.
     *
     * @return The new pose of the robot.
     */
    @Override
    public void update() {
        swerveDriveOdometry.update(drive.getGyroAngle(), drive.getSwerveModuleStates());
    }

    public void updateOdometry(Rotation2d gyroAngle, SwerveModuleState[] swerveModuleStates) {
        swerveDriveOdometry.update(gyroAngle, swerveModuleStates);
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
        resetPosition(pose, drive.getGyroAngle());
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
        instance = new RobotTracker();
    }
}