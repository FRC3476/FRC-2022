package frc.subsystem;

import com.dacubeking.AutoBuilder.robot.sender.pathpreview.RobotPositionSender;
import com.dacubeking.AutoBuilder.robot.sender.pathpreview.RobotState;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.Vector2d;
import frc.robot.Constants;
import frc.subsystem.BlinkinLED.BlinkinLedMode;
import frc.subsystem.BlinkinLED.LedStatus;
import frc.utility.Limelight;
import frc.utility.Limelight.LedMode;
import frc.utility.Limelight.LimelightResolution;
import frc.utility.net.editing.LiveEditableValue;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
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

import static frc.robot.Constants.GOAL_POSITION;
import static frc.robot.Constants.IS_PRACTICE;
import static frc.utility.MathUtil.dist2;

public final class VisionManager extends AbstractSubsystem {
    private static final ReentrantReadWriteLock VISION_MANGER_INSTANCE_LOCK = new ReentrantReadWriteLock();

    private static @Nullable VisionManager instance = null;

    private final @NotNull Limelight limelight = Limelight.getInstance();

    {
        limelight.setCameraResolution(LimelightResolution.k960x720);
    }

    private VisionManager() {
        super(Constants.VISION_MANAGER_PERIOD, 1);
        logData("IS VISION GOOD", true);
    }

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
        Vector3D targetPosition = getPositionOfTargetRelativeToRobot();
        logData("New Distance", Math.hypot(targetPosition.getX(), targetPosition.getZ()));

        Vector2d targetPx = limelight.getTargetPosInCameraPixels();

        logData("py", targetPx.y);
        logData("px", targetPx.x);
    }

    /**
     * @return the current translation of the robot based on the vision data. Will only give correct results if the limelight can
     * see the target
     */
    @Contract(pure = true)
    @NotNull Optional<Translation2d> getVisionTranslation() {
        if (!limelight.isTargetVisible()) return Optional.empty();

        Rotation2d currentGyroAngle = getLatencyCompedLimelightRotation();

        Vector3D offsetVector = getPositionOfTargetRelativeToRobot();
        double angleOffset = Math.atan2(offsetVector.getX(), offsetVector.getZ());


        double distanceToTarget = Units.inchesToMeters(Math.hypot(offsetVector.getX(), offsetVector.getZ()));

        double angleToTarget = currentGyroAngle.getRadians() - angleOffset;
        return Optional.of(new Translation2d(distanceToTarget * Math.cos(angleToTarget),
                distanceToTarget * Math.sin(angleToTarget))
                .plus(GOAL_POSITION));
    }

    private static final Matrix<N3, N3> CAMERA_MATRIX_INVERSE = new MatBuilder<>(Nat.N3(), Nat.N3()).fill(
            0.00392173, 0, -0.6274771,
            0, 0.00389782, -0.46773827,
            0, 0, 1
    );

    private static final MatBuilder<N3, N1> THREE_BY_ONE_MAT_BUILDER = new MatBuilder<>(Nat.N3(), Nat.N1());

    private final LiveEditableValue<Rotation> cameraRotation;

    private final LiveEditableValue<Double> hOffset;
    private final LiveEditableValue<Double> depthOffset;
    private final LiveEditableValue<Vector3D> centerOffset;

    {
        NetworkTable guiTable = limelight.limelightGuiTable;
        hOffset = new LiveEditableValue<>(IS_PRACTICE ? 57.05 : 59.75, guiTable.getEntry("hOffset"));
        depthOffset = new LiveEditableValue<>(IS_PRACTICE ? 32.0 : 14, guiTable.getEntry("depthOffset"));
        centerOffset = new LiveEditableValue<>(new Vector3D(0, 0, IS_PRACTICE ? 6.9 : 18),
                guiTable.getEntry("centerOffset"),
                (value) -> new Vector3D(0, 0, (Double) value),
                Vector3D::getZ);
        cameraRotation = new LiveEditableValue<>(
                new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR,
                        Math.toRadians(IS_PRACTICE ? -37.5 : -34.5), 0, 0),
                guiTable.getEntry("angle"),
                (value) ->
                        new Rotation(
                                RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR,
                                Math.toRadians((Double) value), 0, 0),
                (value) ->
                        Math.toDegrees(value.getAngles(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR)[0])
        );
    }

    private Vector3D getPositionOfTargetRelativeToRobot() {

        Vector2d targetPosInCameraPixels = limelight.getTargetPosInCameraPixels();

        double py = targetPosInCameraPixels.y;
        double px = targetPosInCameraPixels.x;

        Matrix<N3, N1> cameraUnitVector = CAMERA_MATRIX_INVERSE.times(THREE_BY_ONE_MAT_BUILDER.fill(px, py, 1));

        Vector3D vector3D = new Vector3D(
                cameraUnitVector.get(0, 0),
                cameraUnitVector.get(1, 0),
                cameraUnitVector.get(2, 0)
        );

        Vector3D goalDir = cameraRotation.get().applyTo(vector3D);
        double angle = Math.atan2(goalDir.getX(), goalDir.getZ());

        double k = hOffset.get() / goalDir.getY();

        return goalDir.scalarMultiply(k)
                .add(centerOffset.get()) //5.875
                .add(new Vector3D(Math.sin(angle) * depthOffset.get(), 0, Math.cos(angle) * depthOffset.get()));
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
                }
        );
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

        Translation2d relativeGoalPos = ShooterManager.getInstance().getRelativeGoalTranslation();

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


    public int getLoopsWithBadVision() {
        return loopsWithBadVision.get();
    }


    // Shooter methods here for back-compat with autos.
    @Deprecated
    public void shootBalls(double shootTime) throws InterruptedException {
        ShooterManager.getInstance().shootBalls(shootTime);
    }

    @Deprecated
    public void shootBallsUntilBeamBreakHasBroken(double shootTime) throws InterruptedException {
        ShooterManager.getInstance().shootBallsUntilBeamBreakHasBroken(shootTime);
    }


    @Override
    public void close() throws Exception {

    }
}
