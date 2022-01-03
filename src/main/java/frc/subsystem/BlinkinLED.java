package frc.subsystem;

import edu.wpi.first.wpilibj.Spark;

public class BlinkinLED extends AbstractSubsystem {
    private static final BlinkinLED instance = new BlinkinLED();

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
    public void update() {


    }

}
