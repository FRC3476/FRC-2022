package frc.subsystem;

import edu.wpi.first.wpilibj.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Nat;

public class RobotTracker extends AbstractSubsystem {

    private static final RobotTracker trackingInstance = new RobotTracker();
    double startTime;

    Drive drive = Drive.getInstance();

    public static RobotTracker getInstance() {
        return RobotTracker.trackingInstance;
    }

    Pose2d lastEstimatedPose = new Pose2d();

    private SwerveDrivePoseEstimator swerveDriveOdometry;

    private RobotTracker() {
        super(5);
        swerveDriveOdometry = new SwerveDrivePoseEstimator(
            drive.getGyroAngle(),
            new Pose2d(),
            drive.getSwerveDriveKinematics(),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.02),
            new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.02, 0.02, 0.001),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.1),
            5/1000);
    }


    /**
     * @return current rotaion
     */
    public Rotation2d getGyroAngle() {
        return lastEstimatedPose.getRotation();
        
    }

    /**
     * Resets the position on the field to 0,0 with a rotaion of 0 degrees
     */
    synchronized public void resetOdometry() {
        swerveDriveOdometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(drive.getAngle()));
    }

    /**
     * Updates the robot's position on the field using forward kinematics and integration of the pose
     * over time. This method automatically calculates the current time to calculate period
     * (difference between two timestamps). The period is used to calculate the change in distance
     * from a velocity. This also takes in an angle parameter which is used instead of the angular
     * rate that is calculated from forward kinematics.
     *
     * @param gyroAngle The angle reported by the gyroscope.
     * @param moduleStates The current state of all swerve modules. Please provide the states in the
     *     same order in which you instantiated your SwerveDriveKinematics.
     * @return The new pose of the robot.
     */
    @Override
    public void update() {
        swerveDriveOdometry.update(Rotation2d.fromDegrees(drive.getAngle()), drive.getSwerveModuleStates());
        synchronized (this) {
            lastEstimatedPose = swerveDriveOdometry.getEstimatedPosition();
        }
    }

    /**
     * Resets the robot's position on the field.
     * The gyroscope angle does not need to be reset here on the user's robot code. The library automatically takes care of offsetting the gyro angle.
     * 
     * @param pose The position on the field that your robot is at.
     * @param gyroAngle The position on the field that your robot is at.
     */
    synchronized public void resetPosition(Pose2d pose){
        swerveDriveOdometry.resetPosition(pose, drive.getGyroAngle());
    }

    /**
       * Returns the position of the robot on the field.
       *
       * @return The pose of the robot (x and y are in meters).
       */
    public Pose2d getPoseMeters(){
        return lastEstimatedPose;
    }

    @Override
    public void selfTest() {

    }

    @Override
    public void logData() {

    }
}