package frc.subsystem;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import org.jetbrains.annotations.NotNull;

public final class BlinkinLED extends AbstractSubsystem {
    private static @NotNull BlinkinLED instance = new BlinkinLED();

    public static @NotNull BlinkinLED getInstance() {
        return instance;
    }


    @NotNull Spark spark = new Spark(0);

    private BlinkinLED() {
        super(-1);
    }

    public void setColor(double color) {
        spark.set(color);
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
