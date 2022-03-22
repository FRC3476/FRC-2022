package frc.subsystem;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.utility.Timer;
import org.jetbrains.annotations.NotNull;

import java.util.Objects;

public final class BlinkinLED extends AbstractSubsystem {
    private static @NotNull BlinkinLED instance = new BlinkinLED();

    public static @NotNull BlinkinLED getInstance() {
        return instance;
    }


    public static class LedStatus implements Comparable<LedStatus> {
        public final double value;
        public final int priority;

        public LedStatus(BlinkinLedMode value, int priority) {
            this.value = value.value;
            this.priority = priority;
        }

        public LedStatus(double value, int priority) {
            this.value = value;
            this.priority = priority;
        }

        @Override
        public int compareTo(@NotNull BlinkinLED.LedStatus o) {
            return Integer.compare(this.priority, o.priority);
        }

        @Override
        public String toString() {
            return "(" + value + ", " + priority + ")";
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            LedStatus that = (LedStatus) o;
            return Double.compare(that.value, value) == 0 && priority == that.priority;
        }

        @Override
        public int hashCode() {
            return Objects.hash(value, priority);
        }
    }

    public enum BlinkinLedMode {
        // yoinked from https://github.com/Mechanical-Advantage/RobotCode2022/blob/main/src/main/java/frc/robot/util/BlinkinLedDriver.java
        // --- Fixed palette patterns ---
        FIXED_RAINBOW_RAINBOW(-0.99),
        FIXED_RAINBOW_PARTY(-0.97),
        FIXED_RAINBOW_OCEAN(-0.95),
        FIXED_RAINBOW_LAVA(-0.93),
        FIXED_RAINBOW_FOREST(-0.91),
        FIXED_RAINBOW_GLITTER(-0.89),
        FIXED_CONFETTI(-0.87),
        FIXED_SHOT_RED(-0.85),
        FIXED_SHOT_BLUE(-0.83),
        FIXED_SHOT_WHITE(-0.81),
        FIXED_SINELON_RAINBOW(-0.79),
        FIXED_SINELON_PARTY(-0.77),
        FIXED_SINELON_OCEAN(-0.75),
        FIXED_SINELON_LAVA(-0.73),
        FIXED_SINELON_FOREST(-0.71),
        FIXED_BEATS_RAINBOW(-0.69),
        FIXED_BEATS_PARTY(-0.67),
        FIXED_BEATS_OCEAN(-0.65),
        FIXED_BEATS_LAVA(-0.63),
        FIXED_BEATS_FOREST(-0.61),
        FIXED_FIRE_MEDIUM(-0.59),
        FIXED_FIRE_LARGE(-0.57),
        FIXED_TWINKLES_RAINBOW(-0.55),
        FIXED_TWINKLES_PARTY(-0.53),
        FIXED_TWINKLES_OCEAN(-0.51),
        FIXED_TWINKLES_LAVA(-0.49),
        FIXED_TWINKLES_FOREST(-0.47),
        FIXED_WAVES_RAINBOW(-0.45),
        FIXED_WAVES_PARTY(-0.43),
        FIXED_WAVES_OCEAN(-0.41),
        FIXED_WAVES_LAVA(-0.39),
        FIXED_WAVES_FOREST(-0.37),
        FIXED_LARSON_RED(-0.35),
        FIXED_LARSON_GRAY(-0.33),
        FIXED_CHASE_RED(-0.31),
        FIXED_CHASE_BLUE(-0.29),
        FIXED_CHASE_GRAY(-0.27),
        FIXED_HEARTBEAT_RED(-0.25),
        FIXED_HEARTBEAT_BLUE(-0.23),
        FIXED_HEARTBEAT_WHITE(-0.21),
        FIXED_HEARTBEAT_GRAY(-0.19),
        FIXED_BREATH_RED(-0.17),
        FIXED_BREATH_BLUE(-0.15),
        FIXED_BREATH_GRAY(-0.13),
        FIXED_STROBE_RED(-0.11),
        FIXED_STROBE_BLUE(-0.09),
        FIXED_STROBE_GOLD(-0.07),
        FIXED_STROBE_WHITE(-0.05),

        // --- Color one patterns ---
        ONE_BLEND_TO_BLACK(-0.03),
        ONE_LARSON(-0.01),
        ONE_CHASE(0.01),
        ONE_HEARTBEAT_SLOW(0.03),
        ONE_HEARTBEAT_MEDIUM(0.05),
        ONE_HEARTBEAT_FAST(0.07),
        ONE_BREATH_SLOW(0.09),
        ONE_BREATH_FAST(0.11),
        ONE_SHOT(0.13),
        ONE_STROBE(0.15),

        // --- Color two patterns ---
        TWO_BLEND_TO_BLACK(0.17),
        TWO_LARSON(0.19),
        TWO_CHASE(0.21),
        TWO_HEARTBEAT_SLOW(0.23),
        TWO_HEARTBEAT_MEDIUM(0.25),
        TWO_HEARTBEAT_FAST(0.27),
        TWO_BREATH_SLOW(0.29),
        TWO_BREATH_FAST(0.31),
        TWO_SHOT(0.33),
        TWO_STROBE(0.35),

        // --- Combined color patterns ---
        BOTH_SPARKLE_ONE_ON_TWO(0.37),
        BOTH_SPARKLE_TWO_ON_ONE(0.39),
        BOTH_GRADIENT(0.41),
        BOTH_BEATS(0.43),
        BOTH_BLEND_ONE_TO_TWO(0.45),
        BOTH_BLEND(0.47),
        BOTH_NO_BLEND(0.49),
        BOTH_TWINKLES(0.51),
        BOTH_WAVES(0.53),
        BOTH_SINELON(0.55),

        // --- Solid colors ---
        SOLID_HOT_PINK(0.57),
        SOLID_DARK_RED(0.59),
        SOLID_RED(0.61),
        SOLID_RED_ORANGE(0.63),
        SOLID_ORANGE(0.65),
        SOLID_GOLD(0.67),
        SOLID_YELLOW(0.69),
        SOLID_LAWN_GREEN(0.71),
        SOLID_LIME(0.73),
        SOLID_DARK_GREEN(0.75),
        SOLID_GREEN(0.77),
        SOLID_BLUE_GREEN(0.79),
        SOLID_AQUA(0.81),
        SOLID_SKY_BLUE(0.83),
        SOLID_DARK_BLUE(0.85),
        SOLID_BLUE(0.87),
        SOLID_BLUE_VIOLET(0.89),
        SOLID_VIOLET(0.91),
        SOLID_WHITE(0.93),
        SOLID_GRAY(0.95),
        SOLID_DARK_GRAY(0.97),
        SOLID_BLACK(0.99);

        private final double value;

        BlinkinLedMode(double value) {
            this.value = value;
        }
    }

    @NotNull final Spark spark;

    private BlinkinLED() {
        super(-1);
        spark = new Spark(0);
    }

    private @NotNull LedStatus currentStatus = new LedStatus(BlinkinLedMode.SOLID_ORANGE, -100);

    private double statusExpiryTime = 0;

    /**
     * Set the status of the BlinkinLED.
     * <p>
     * If the ledStatus has a priority that is >= the current priority the current status is set as the LEDs state. OR if the
     * previous ledStatus has expired the current ledStatus is set as the LED's state.
     */
    public synchronized void setStatus(LedStatus ledStatus) {
        if (ledStatus.priority >= currentStatus.priority || Timer.getFPGATimestamp() > statusExpiryTime) {
            spark.set(ledStatus.value);
            currentStatus = ledStatus;
            statusExpiryTime = Timer.getFPGATimestamp() + 0.1;
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
        spark.close();
        instance = new BlinkinLED();
    }
}
