package frc.subsystem;

import com.dacubeking.AutoBuilder.robot.sender.pathpreview.RobotPositionSender;
import com.dacubeking.AutoBuilder.robot.sender.pathpreview.RobotState;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import frc.utility.CircleFitter;
import frc.utility.Limelight;
import frc.utility.Limelight.LedMode;
import frc.utility.Limelight.LimelightResolution;
import frc.utility.geometry.GeometryUtils;
import frc.utility.net.editing.LiveEditableValue;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import static frc.robot.Constants.*;
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
    private final LiveEditableValue<Rotation2d> cameraRotation2d;
    private final LiveEditableValue<Double> hOffset;
    private final LiveEditableValue<Double> depthOffset;
    private final LiveEditableValue<Vector3D> centerOffset;
    private final LiveEditableValue<Transform2d> centerOffsetTranslation2d;

    {
        NetworkTable guiTable = limelight.limelightGuiTable;
        hOffset = new LiveEditableValue<>(IS_PRACTICE ? 57.05 : 59.75, guiTable.getEntry("hOffset"));
        depthOffset = new LiveEditableValue<>(IS_PRACTICE ? 32.0 : 14, guiTable.getEntry("depthOffset"));
        centerOffset = new LiveEditableValue<>(new Vector3D(0, 0, IS_PRACTICE ? 6.9 : 18),
                guiTable.getEntry("centerOffset"),
                (value) -> new Vector3D(0, 0, (Double) value),
                Vector3D::getZ);

        centerOffsetTranslation2d = new LiveEditableValue<>(
                new Transform2d(new Translation2d(IS_PRACTICE ? 6.9 : 18, 0), new Rotation2d()),
                guiTable.getEntry("centerOffsetNew"),
                (value) -> new Transform2d(new Translation2d((Double) value, 0), new Rotation2d()),
                Transform2d::getX);

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

        cameraRotation2d = new LiveEditableValue<>(
                new Rotation2d(Math.toRadians(IS_PRACTICE ? -37.5 : -34.5)),
                guiTable.getEntry("angle"),
                (value) -> Rotation2d.fromDegrees((Double) value),
                Rotation2d::getDegrees
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


        Optional<Pose2d> circleFitVisionPoseOptional = processFrame();

        if (circleFitVisionPoseOptional.isPresent()) {
            Pose2d circleFitVisionPose = circleFitVisionPoseOptional.get();

            if (DriverStation.isTeleopEnabled()) {
                robotTracker.addVisionMeasurement(circleFitVisionPose.getTranslation(), getLimelightTime(), false);
            }
            RobotPositionSender.addRobotPosition(new RobotState(
                    circleFitVisionPose.getX(),
                    circleFitVisionPose.getY(),
                    circleFitVisionPose.getRotation().getRadians(),
                    getLimelightTime(),
                    "Circle Fit Vision Pose"
            ));

            logData("Circle Fit Vision Pose X", circleFitVisionPose.getX());
            logData("Circle Fit Vision Pose Y", circleFitVisionPose.getY());
            logData("Circle Fit Vision Pose Angle", circleFitVisionPose.getRotation().getRadians());
            logData("Circle Fit Vision Pose Time", getLimelightTime());
        }

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
                double positionError =
                        dist2(robotTracker.getPoseAtTime(limelight.getTimestamp()).orElseGet(Pose2d::new).getTranslation(),
                                robotTranslation);
                if ((limelight.getCorners().length % 4 == 0 || positionError < Constants.VISION_MANAGER_DISTANCE_THRESHOLD_SQUARED)
                        && limelight.getCorners().length >= MIN_CORNERS && limelight.getCorners().length <= MAX_CORNERS) {
                    if (DriverStation.isTeleopEnabled()) {
                        // robotTracker.addVisionMeasurement(robotTranslation, getLimelightTime(), false);
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

    double lastCaptureTimestamp = 0;
    private static final int MIN_TARGET_COUNT = 3; // For calculating odometry
    private double lastTranslationsTimestamp;
    private List<Translation2d> lastTranslations;

    public static final double VISION_TARGET_HEIGHT_LOWER =
            Units.inchesToMeters(8.0 * 12.0 + 5.625); // Bottom of tape
    public static final double VISION_TARGET_HEIGHT_UPPER = VISION_TARGET_HEIGHT_LOWER + Units.inchesToMeters(2.0); // Top of tape
    public static final double VISION_TARGET_DIAMETER = Units.inchesToMeters(4.0 * 12.0 + 5.375);
    private static final double CIRCLE_FIT_PRECISION = 0.01;

    public static final double FIELD_LENGTH = Units.inchesToMeters(54.0 * 12.0);
    public static final double FIELD_WIDTH = Units.inchesToMeters(27.0 * 12.0);
    public static final Translation2d HUB_CENTER = new Translation2d(FIELD_LENGTH / 2.0, FIELD_WIDTH / 2.0);

    /**
     * Process the current vision data
     *
     * @return The robot's position relative to the field
     * @author jonahb55
     * @author Aum-P
     * @author V-RA
     * @author Ayushk2023
     * @author Mechanical Advantage 6328
     */
    private Optional<Pose2d> processFrame() {
        // noinspection FloatingPointEquality
        if (Limelight.getInstance().getTimestamp() == lastCaptureTimestamp) {
            // Exit if no new frame
            return Optional.empty();
        }

        Limelight.getInstance().getTimestamp();
        lastCaptureTimestamp = Limelight.getInstance().getTimestamp();

        CameraPosition cameraPosition = new CameraPosition(hOffset.get(), cameraRotation2d.get(),
                centerOffsetTranslation2d.get());
        Vector2d[] cornersRaw = Limelight.getInstance().getCorners();

        int targetCount = cornersRaw.length / 4;
        // Calculate camera to target translation
        if (targetCount >= MIN_TARGET_COUNT && cornersRaw.length % 4 == 0) {
            // Calculate individual corner translations
            List<Translation2d> cameraToTargetTranslations = new ArrayList<>();
            for (int targetIndex = 0; targetIndex < targetCount; targetIndex++) {
                List<VisionPoint> corners = new ArrayList<>();
                double totalX = 0.0, totalY = 0.0;
                for (int i = targetIndex * 4; i < (targetIndex * 4) + 4; i++) {
                    if (i < cornersRaw.length) {
                        corners.add(new VisionPoint(cornersRaw[i].x, cornersRaw[i].y));
                        totalX += cornersRaw[i].x;
                        totalY += cornersRaw[i].y;
                    }
                }

                VisionPoint targetAvg = new VisionPoint(totalX / 4, totalY / 4);
                corners = sortCorners(corners, targetAvg);

                for (int i = 0; i < corners.size(); i++) {
                    Translation2d translation = solveCameraToTargetTranslation(
                            corners.get(i), i < 2 ? VISION_TARGET_HEIGHT_LOWER : VISION_TARGET_HEIGHT_UPPER, cameraPosition);
                    if (translation != null) {
                        cameraToTargetTranslations.add(translation);
                    }
                }
            }

            // Save individual translations
            lastTranslationsTimestamp = Timer.getFPGATimestamp();
            lastTranslations = cameraToTargetTranslations;

            // Combine corner translations to full target translation
            if (cameraToTargetTranslations.size() >= MIN_TARGET_COUNT * 4) {
                Translation2d cameraToTargetTranslation =
                        CircleFitter.fit(VISION_TARGET_DIAMETER / 2.0,
                                cameraToTargetTranslations, CIRCLE_FIT_PRECISION);

                // Calculate field to robot translation
                Rotation2d robotRotation = RobotTracker.getInstance().getGyroRotation(lastCaptureTimestamp);
                Rotation2d cameraRotation = robotRotation
                        .rotateBy(cameraPosition.vehicleToCamera.getRotation());
                Transform2d fieldToTargetRotated =
                        new Transform2d(HUB_CENTER, cameraRotation);
                Transform2d fieldToCamera = fieldToTargetRotated.plus(
                        GeometryUtils.transformFromTranslation(cameraToTargetTranslation.unaryMinus()));
                Pose2d fieldToVehicle =
                        GeometryUtils.transformToPose(fieldToCamera.plus(cameraPosition.vehicleToCamera.inverse()));

                if (fieldToVehicle.getX() > FIELD_LENGTH
                        || fieldToVehicle.getX() < 0.0
                        || fieldToVehicle.getY() > FIELD_WIDTH
                        || fieldToVehicle.getY() < 0.0) {
                    return Optional.empty();
                }
                return Optional.of(fieldToVehicle);
            }
        }
        return Optional.empty();
    }

    /**
     * @author jonahb55
     * @author Aum-P
     * @author V-RA
     * @author Ayushk2023
     * @author Mechanical Advantage 6328
     */
    private List<VisionPoint> sortCorners(List<VisionPoint> corners,
                                          VisionPoint average) {

        // Find top corners
        Integer topLeftIndex = null;
        Integer topRightIndex = null;
        double minPosRads = Math.PI;
        double minNegRads = Math.PI;
        for (int i = 0; i < corners.size(); i++) {
            VisionPoint corner = corners.get(i);
            double angleRad =
                    new Rotation2d(corner.x - average.x, average.y - corner.y)
                            .minus(Rotation2d.fromDegrees(90)).getRadians();
            if (angleRad > 0) {
                if (angleRad < minPosRads) {
                    minPosRads = angleRad;
                    topLeftIndex = i;
                }
            } else {
                if (Math.abs(angleRad) < minNegRads) {
                    minNegRads = Math.abs(angleRad);
                    topRightIndex = i;
                }
            }
        }

        // Find lower corners
        Integer lowerIndex1 = null;
        Integer lowerIndex2 = null;
        for (int i = 0; i < corners.size(); i++) {
            boolean alreadySaved = false;
            if (topLeftIndex != null) {
                if (topLeftIndex.equals(i)) {
                    alreadySaved = true;
                }
            }
            if (topRightIndex != null) {
                if (topRightIndex.equals(i)) {
                    alreadySaved = true;
                }
            }
            if (!alreadySaved) {
                if (lowerIndex1 == null) {
                    lowerIndex1 = i;
                } else {
                    lowerIndex2 = i;
                }
            }
        }

        // Combine final list
        List<VisionPoint> newCorners = new ArrayList<>();
        if (topLeftIndex != null) {
            newCorners.add(corners.get(topLeftIndex));
        }
        if (topRightIndex != null) {
            newCorners.add(corners.get(topRightIndex));
        }
        if (lowerIndex1 != null) {
            newCorners.add(corners.get(lowerIndex1));
        }
        if (lowerIndex2 != null) {
            newCorners.add(corners.get(lowerIndex2));
        }
        return newCorners;
    }

    public static final Rotation2d FOV_HORIZONTAL = Rotation2d.fromDegrees(59.6);
    public static final Rotation2d FOV_VERTICAL = Rotation2d.fromDegrees(49.7);

    private static final double vpw = 2.0 * Math.tan(FOV_HORIZONTAL.getRadians() / 2.0);
    private static final double vph = 2.0 * Math.tan(FOV_VERTICAL.getRadians() / 2.0);

    /**
     * Camera offsets in pixels?
     */
    public static final double CROSSHAIR_X = 0;
    public static final double CROSSHAIR_Y = 0;

    /**
     * @author jonahb55
     * @author Aum-P
     * @author V-RA
     * @author Ayushk2023
     * @author Mechanical Advantage 6328
     */
    private Translation2d solveCameraToTargetTranslation(VisionPoint corner, double goalHeight, CameraPosition cameraPosition) {

        double halfWidthPixels = limelight.getCameraResolution().width / 2.0;
        double halfHeightPixels = limelight.getCameraResolution().height / 2.0;
        double nY = -((corner.x - halfWidthPixels - CROSSHAIR_X) / halfWidthPixels);
        double nZ = -((corner.y - halfHeightPixels - CROSSHAIR_Y) / halfHeightPixels);

        Translation2d xzPlaneTranslation = new Translation2d(1.0, vph / 2.0 * nZ).rotateBy(cameraPosition.verticalRotation);
        double x = xzPlaneTranslation.getX();
        double y = vpw / 2.0 * nY;
        double z = xzPlaneTranslation.getY();

        double differentialHeight = cameraPosition.cameraHeight - goalHeight;
        if ((z < 0.0) == (differentialHeight > 0.0)) {
            double scaling = differentialHeight / -z;
            double distance = Math.hypot(x, y) * scaling;
            Rotation2d angle = new Rotation2d(x, y);
            return new Translation2d(distance * angle.getCos(), distance * angle.getSin());
        }
        return null;
    }

    /**
     * @author jonahb55
     */
    public static class VisionPoint {
        public final double x;
        public final double y;

        public VisionPoint(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    /**
     * @author jonahb55
     */
    public static final class CameraPosition {
        public final double cameraHeight;
        public final Rotation2d verticalRotation;
        public final Transform2d vehicleToCamera;

        public CameraPosition(double cameraHeight, Rotation2d verticalRotation,
                              Transform2d vehicleToCamera) {
            this.cameraHeight = cameraHeight;
            this.verticalRotation = verticalRotation;
            this.vehicleToCamera = vehicleToCamera;
        }
    }
}
