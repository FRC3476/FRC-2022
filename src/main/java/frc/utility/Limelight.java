package frc.utility;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import org.jetbrains.annotations.NotNull;

/**
 * This class is used to get data from the limelight network tables
 */
public final class Limelight {
    final @NotNull NetworkTable limelightTable;
    final @NotNull NetworkTable limelightGuiTable;

    private static final Limelight INSTANCE = new Limelight();

    public static @NotNull Limelight getInstance() {
        return INSTANCE;
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

    @SuppressWarnings("SpellCheckingInspection")
    private Limelight() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        limelightGuiTable = NetworkTableInstance.getDefault().getTable("limelightgui");
        limelightGuiTable.getEntry("CameraTargetHeightOffset").setDouble(Constants.CAMERA_TARGET_HEIGHT_OFFSET);
        limelightGuiTable.getEntry("CameraYAngle").setDouble(Constants.CAMERA_Y_ANGLE);

        limelightTable.getEntry("tl").addListener(event -> lastUpdate = Timer.getFPGATimestamp(),
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        limelightGuiTable.getEntry("forceledon").addListener(event -> {
            if (event.getEntry().getBoolean(false)) {
                INSTANCE.setLedMode(LedMode.ON);
                INSTANCE.setCamMode(CamMode.VISION_PROCESSOR);
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
        return limelightTable.getEntry("tx").getDouble(0);
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
        return limelightTable.getEntry("ts").getDouble(0);
    }

    /**
     * Sets limelight’s LED state
     */
    public void setLedMode(@NotNull LedMode ledMode) {
        if (limelightGuiTable.getEntry("forceledon").getBoolean(false)) {
            limelightTable.getEntry("ledMode").setNumber(3);
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
            return Units.inchesToMeters((Constants.CAMERA_TARGET_HEIGHT_OFFSET) / Math.tan(
                    Math.toRadians(Constants.CAMERA_Y_ANGLE + getVerticalOffset())));
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
            return (Constants.CAMERA_TARGET_HEIGHT_OFFSET) / Math.tan(
                    Math.toRadians(Constants.CAMERA_Y_ANGLE + getVerticalOffset()));
        } else {
            return 0;
        }
    }
}