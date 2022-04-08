package frc.utility;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.Vector2d;
import frc.subsystem.AbstractSubsystem;
import frc.utility.net.editing.LiveEditableValue;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.jetbrains.annotations.NotNull;

import java.util.HashMap;
import java.util.concurrent.locks.ReentrantReadWriteLock;

/**
 * This class is used to get data from the limelight network tables
 */
public final class Limelight extends AbstractSubsystem {
    public static final double[] EMPTY_DOUBLE_ARRAY = new double[0];
    final @NotNull NetworkTable limelightTable;
    final @NotNull NetworkTable limelightGuiTable;

    private static final HashMap<String, Limelight> LIMELIGHT_MAP = new HashMap<>();
    private static final ReentrantReadWriteLock LIMELIGHT_MAP_LOCK = new ReentrantReadWriteLock();

    public static @NotNull Limelight getInstance() {
        return getInstance("limelight");
    }


    private final LiveEditableValue<Rotation> cameraRotation;

    private final LiveEditableValue<Double> hOffset;
    private final LiveEditableValue<Double> depthOffset;
    private final LiveEditableValue<Vector3D> centerOffset;

    public static @NotNull Limelight getInstance(String name) {
        LIMELIGHT_MAP_LOCK.readLock().lock();
        try {
            if (LIMELIGHT_MAP.containsKey(name)) {
                return LIMELIGHT_MAP.get(name);
            }
        } finally {
            LIMELIGHT_MAP_LOCK.readLock().unlock();
        }

        LIMELIGHT_MAP_LOCK.writeLock().lock();
        try {
            if (LIMELIGHT_MAP.containsKey(name)) {
                return LIMELIGHT_MAP.get(name);
            } else {
                Limelight limelight = new Limelight(name);
                LIMELIGHT_MAP.put(name, limelight);
                return limelight;
            }
        } finally {
            LIMELIGHT_MAP_LOCK.writeLock().unlock();
        }
    }

    @Override
    public void selfTest() {

    }

    @Override
    public void logData() {

    }

    @Override
    public void close() throws Exception {

    }


    /**
     * Limelight’s LED states
     */
    public enum LedMode {
        /**
         * use mode set in the current pipeline.
         */
        DEFAULT(0),
        /**
         * Force off
         */
        OFF(1),
        /**
         * Force blinking
         */
        BLINK(2),
        /**
         * Force on
         */
        ON(3);

        LedMode(int i) {
            this.i = i;
        }

        private final int i;
    }

    /**
     * limelight’s operation modes
     */
    public enum CamMode {
        VISION_PROCESSOR(0),
        /**
         * Increases exposure, disables vision processing
         */
        DRIVER_CAMERA(1);

        private final int i;

        CamMode(int i) {
            this.i = i;
        }
    }

    /**
     * limelight’s streaming modes
     */
    public enum StreamingMode {
        /**
         * Side-by-side streams if a webcam is attached to Limelight
         */
        STANDARD(0),
        /**
         * The secondary camera stream is placed in the lower-right corner of the primary camera stream
         */
        PIP_MAIN(1),
        /**
         * The primary camera stream is placed in the lower-right corner of the secondary camera stream
         */
        PIP_SECONDARY(2);

        private final int i;

        StreamingMode(int i) {
            this.i = i;
        }
    }

    private Limelight(String name) {
        super(-1);
        limelightTable = NetworkTableInstance.getDefault().getTable(name);
        limelightGuiTable = NetworkTableInstance.getDefault().getTable(name + "gui");

        cameraRotation = new LiveEditableValue<>(
                new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, Math.toRadians(-38), 0, 0),
                limelightGuiTable.getEntry("angle"),
                (value) ->
                        new Rotation(
                                RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR,
                                Math.toRadians((Double) value), 0, 0),
                (value) ->
                        value.getAngles(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR)[0]
        );
        hOffset = new LiveEditableValue<>(38.0, limelightGuiTable.getEntry("hOffset"));
        depthOffset = new LiveEditableValue<>(44.0, limelightGuiTable.getEntry("depthOffset"));
        centerOffset = new LiveEditableValue<>(new Vector3D(0, 0, 6.9),
                limelightGuiTable.getEntry("centerOffset"),
                (value) -> new Vector3D(0, 0, (Double) value),
                Vector3D::getZ);


        limelightTable.getEntry("tl").addListener(event -> lastUpdate = Timer.getFPGATimestamp(),
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        limelightGuiTable.getEntry("forceledon").addListener(event -> {
            if (event.getEntry().getBoolean(false)) {
                this.setLedMode(LedMode.ON);
                this.setCamMode(CamMode.VISION_PROCESSOR);
            }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }


    /**
     * @return Whether the limelight has any valid targets
     */
    public boolean isTargetVisible() {
        return limelightTable.getEntry("tv").getDouble(0) == 1 && isConnected();
    }

    double lastUpdate = 0;

    public boolean isConnected() {
        //System.out.println(Timer.getFPGATimestamp() - lastUpdate);
        return Timer.getFPGATimestamp() - lastUpdate < 2;
    }

    /**
     * @return Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
     */
    public double getHorizontalOffset() {
        return limelightTable.getEntry("tx").getDouble(0) - 1;
    }

    /**
     * @return Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
     */
    public double getVerticalOffset() {
        return limelightTable.getEntry("ty").getDouble(0);
    }

    /**
     * @return Target Area (0% of image to 100% of image)
     */
    public double getTargetArea() {
        return limelightTable.getEntry("ta").getDouble(0);
    }

    /**
     * @return Skew or rotation (-90 degrees to 0 degrees)
     */
    public double getTargetSkew() {
        return limelightTable.getEntry("ts").getDouble(0);
    }

    /**
     * @return The pipeline’s latency contribution (ms). Add at least 11ms for image capture latency.
     */
    public double getLatency() {
        return limelightTable.getEntry("tl").getDouble(0);
    }

    /**
     * @return The timeStamp of the last processed frame
     */
    public double getTimestamp() {
        return lastUpdate - (getLatency() + 11) / 1000;
    }

    /**
     * Sets limelight’s LED state
     */
    public void setLedMode(@NotNull LedMode ledMode) {
        if (limelightGuiTable.getEntry("forceledon").getBoolean(false)) {
            limelightTable.getEntry("ledMode").setNumber(LedMode.ON.i);
        } else {
            limelightTable.getEntry("ledMode").setNumber(ledMode.i);
        }
    }

    /**
     * Sets limelight’s operation mode
     */
    public void setCamMode(@NotNull CamMode camMode) {
        if (limelightGuiTable.getEntry("forceledon").getBoolean(false)) {
            limelightTable.getEntry("camMode").setNumber(0);
        } else {
            limelightTable.getEntry("camMode").setNumber(camMode.i);
        }
    }

    /**
     * Sets limelight’s current pipeline
     *
     * @param pipeline Select pipeline 0...9
     */
    public void setPipeline(int pipeline) {
        limelightTable.getEntry("pipeline").setNumber(pipeline);
    }

    /**
     * Sets limelight’s streaming mode
     */
    public void setStreamingMode(@NotNull StreamingMode streamingMode) {
        limelightTable.getEntry("stream").setNumber(streamingMode.i);
    }

    /**
     * Allows users to take snapshots during a match
     *
     * @param takeSnapshots Take two snapshots per second
     */
    public void takeSnapshots(boolean takeSnapshots) {
        if (takeSnapshots) {
            limelightTable.getEntry("snapshot").setNumber(1);
        } else {
            limelightTable.getEntry("snapshot").setNumber(0);
        }
    }

    /**
     * @return Distance from the limelight to the target in Meters
     * @see <a href="https://docs.limelightvision.io/en/latest/cs_estimating_distance.html">Limelight Docs: Estimating
     * Distance</a>
     */
    public double getDistanceM() {
        if (isTargetVisible()) {
            return Units.inchesToMeters(getDistance());
        } else {
            return 0;
        }
    }


    /**
     * @return Distance from the limelight to the target in IN
     * @see <a href="https://docs.limelightvision.io/en/latest/cs_estimating_distance.html">Limelight Docs: Estimating
     * Distance</a>
     */
    public double getDistance() {
        if (isTargetVisible()) {
            return 68.728 / (Math.tan(Math.toRadians(57.952 + getVerticalOffset())) - 0.324129);
        } else {
            return 0;
        }
    }

    public Vector2d getTargetPosInCameraPixels() {
        return new Vector2d(
                (getHorizontalOffset() / 29.8) * (320.0 / 2) + (320.0 / 2),
                (getVerticalOffset() / 24.85) * (240.0 / 2) + (240.0 / 2)
        );
    }

    private static final Matrix<N3, N3> CAMERA_MATRIX_INVERSE = new MatBuilder<>(Nat.N3(), Nat.N3()).fill(
            0.00392173, 0, -0.6274771,
            0, 0.00389782, -0.46773827,
            0, 0, 1
    );

    private static final MatBuilder<N3, N1> THREE_BY_ONE_MAT_BUILDER = new MatBuilder<>(Nat.N3(), Nat.N1());
    /**
     * @return The Correct distance from the limelight to the target in IN. This correctly compensates for the angle of the camera
     * to ensure a consistent distance as the robot rotates.
     */
    public Vector3D getCorrectTargetVector() {
        if (isTargetVisible()) {

            Vector2d targetPosInCameraPixels = getTargetPosInCameraPixels();

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
        } else {
            return Vector3D.ZERO;
        }
    }

    public Vector2d[] getCorners() {
        double[] corners = limelightTable.getEntry("tcornxy").getDoubleArray(EMPTY_DOUBLE_ARRAY);
        Vector2d[] processedCorners = new Vector2d[corners.length / 2];
        for (int i = 0; i < corners.length; i += 2) {
            if (i + 1 < corners.length) {
                processedCorners[i / 2] = new Vector2d(corners[i], corners[i + 1]);
            }
        }
        return processedCorners;
    }

    public boolean areCornersTouchingEdge() {
        Vector2d[] corners = getCorners();
        for (Vector2d corner : corners) {
            if (corner.x < 15 || corner.x > 320 - 15 || corner.y < 0 || corner.y > 240 - 15) {

                return true;
            }
        }
        return false;
    }
}