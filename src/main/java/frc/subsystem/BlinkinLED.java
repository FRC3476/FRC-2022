package frc.subsystem;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public final class BlinkinLED extends AbstractSubsystem {
    private static BlinkinLED instance = new BlinkinLED();

    public static BlinkinLED getInstance() {
        return instance;
    }


    Spark spark = new Spark(0);

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
