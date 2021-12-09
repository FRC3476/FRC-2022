package frc.utility;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/**
 * This class is used to get data from the limelight network tables
 */
public class Limelight {
    NetworkTable limelightTable;
    NetworkTable limelightGuiTable;

    private static final Limelight limelight = new Limelight();

    public static Limelight getInstance() {
        return limelight;
    }

    /**
     * Limelight’s LED states
     */
    public enum LedMode {
        /**
         * use mode set in the current pipeline.
         */
        DEFAULT,
        /**
         * Force off
         */
        OFF,
        /**
         * Force blinking
         */
        BLINK,
        /**
         * Force on
         */
        ON
    }

    /**
     * limelight’s operation modes
     */
    public enum CamMode {
        VISION_PROCESSOR,
        /**
         * Increases exposure, disables vision processing
         */
        DRIVER_CAMERA
    }

    /**
     * limelight’s streaming modes
     */
    public enum StreamingMode {
        /**
         * Side-by-side streams if a webcam is attached to Limelight
         */
        STANDARD,
        /**
         * The secondary camera stream is placed in the lower-right corner of the primary camera stream
         */
        PIP_MAIN,
        /**
         * The primary camera stream is placed in the lower-right corner of the secondary camera stream
         */
        PIP_SECONDARY
    }

    private Limelight() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        limelightGuiTable = NetworkTableInstance.getDefault().getTable("limelightgui");
        limelightGuiTable.getEntry("CameraTargetHeightOffset").setDouble(Constants.CAMERA_TARGET_HEIGHT_OFFSET);
        limelightGuiTable.getEntry("CameraYAngle").setDouble(Constants.CAMERA_Y_ANGLE);

        limelightTable.getEntry("tl").addListener(event -> {
            lastUpdate = Timer.getFPGATimestamp();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        limelightGuiTable.getEntry("forceledon").addListener(event -> {
            if (event.getEntry().getBoolean(false)) {
                limelight.setLedMode(LedMode.ON);
            }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }


    /**
     * @return Whether the limelight has any valid targets
     */
    public boolean isTargetVisiable() {
		return limelightTable.getEntry("tv").getDouble(0) == 1;
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
    public double getTagetArea() {
        return limelightTable.getEntry("ta").getDouble(0);
    }

    /**
     * @return Skew or rotation (-90 degrees to 0 degrees)
     */
    public double getTagetSkew() {
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
     *
     * @param ledMode
     */
    public void setLedMode(LedMode ledMode) {
        if (!limelightGuiTable.getEntry("forceledon").getBoolean(false)) {
            switch (ledMode) {
                case DEFAULT:
                    limelightTable.getEntry("ledMode").setNumber(0);
                    break;
                case OFF:
                    limelightTable.getEntry("ledMode").setNumber(1);
                    break;
                case BLINK:
                    limelightTable.getEntry("ledMode").setNumber(2);
                    break;
                case ON:
                    limelightTable.getEntry("ledMode").setNumber(3);
                    break;
            }
        } else {
            limelightTable.getEntry("ledMode").setNumber(3);
        }

    }

    /**
     * Sets limelight’s operation mode
     *
     * @param camMode
     */
    public void setCamMode(CamMode camMode) {
        switch (camMode) {
            case VISION_PROCESSOR:
                limelightTable.getEntry("camMode").setNumber(0);
                break;
            case DRIVER_CAMERA:
                limelightTable.getEntry("camMode").setNumber(1);
                break;
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
     *
     * @param streamingMode
     */
    public void setSreamingMode(StreamingMode streamingMode) {
        switch (streamingMode) {
            case STANDARD:
                limelightTable.getEntry("stream").setNumber(0);
                break;
            case PIP_MAIN:
                limelightTable.getEntry("stream").setNumber(1);
                break;
            case PIP_SECONDARY:
                limelightTable.getEntry("stream").setNumber(2);
                break;
        }
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
     * @return Distance from the limelight to the target in cm
     * @see https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
     */
    public double getDistance() {
        if (isTargetVisiable()) {
            return (Constants.CAMERA_TARGET_HEIGHT_OFFSET) / Math.tan(
                    Math.toRadians(Constants.CAMERA_Y_ANGLE + getVerticalOffset()));
        } else {
            return 0;
        }
    }


}