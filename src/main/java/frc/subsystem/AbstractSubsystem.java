package frc.subsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.jetbrains.annotations.NotNull;

import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.Map;

@SuppressWarnings("unused")
public abstract class AbstractSubsystem implements Runnable, AutoCloseable {
    private final int period;
    private final int loggingInterval;
    private int logInterval;
    private @NotNull ThreadSignal signal = ThreadSignal.PAUSED;
    public String subsystemName;

    public enum ThreadSignal {
        ALIVE, PAUSED, DEAD
    }

    /**
     * @param period The period when calling update
     */
    public AbstractSubsystem(int period, int loggingInterval) {
        this.period = period;
        this.subsystemName = this.getClass().getSimpleName();
        this.loggingInterval = loggingInterval;
    }

    public AbstractSubsystem(int period) {
        this(period, 2);
    }

    public abstract void selfTest();

    public abstract void logData();

    public void pause() {
        signal = ThreadSignal.PAUSED;
    }

    public void kill() {
        signal = ThreadSignal.DEAD;
    }

    public void start() {
        signal = ThreadSignal.ALIVE;
    }

    /**
     * This function will be called repeatedly when the thread is alive. The period will be whatever you defined when creating the
     * object
     */
    public void update() {

    }

    static Map<String, Object> logDataMap = new HashMap<>();


    public void logData(String key, Object value) {
        //SmartDashboard.putString(key, value.toString());
        logDataMap.put(key, value);
    }

    int lastLength = 20;

    public void pushLog() {
        StringBuilder sb = new StringBuilder((int) (lastLength * 1.5));

        for (Map.Entry<String, Object> entry : logDataMap.entrySet()) {
            sb.append(entry.getKey()).append(":").append(entry.getValue()).append("\n");
        }
        lastLength = sb.length();
        SmartDashboard.putRaw(subsystemName, sb.toString().getBytes(StandardCharsets.ISO_8859_1));
    }

    @Override
    @SuppressWarnings("BusyWait")
    public void run() {
        while (signal != ThreadSignal.DEAD) {
            double startTime = Timer.getFPGATimestamp();
            if (signal == ThreadSignal.ALIVE) {
                update();
                logInterval++;
                if (logInterval >= loggingInterval) {
                    logData();
                    logInterval = 1;
                }
            }
            double executionTimeMS = (Timer.getFPGATimestamp() - startTime) * 1000;
            try {
                if (period - executionTimeMS > 0) {
                    Thread.sleep((long) (period - executionTimeMS));
                }
            } catch (Exception e) {
                System.out.println("Thread sleep failing " + subsystemName + " message: " + e.getMessage());
            }

        }
    }
}
