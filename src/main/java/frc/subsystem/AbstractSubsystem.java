package frc.subsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import org.jetbrains.annotations.NotNull;

import java.util.HashMap;
import java.util.Map;

@SuppressWarnings("unused")
public abstract class AbstractSubsystem implements Runnable, AutoCloseable {
    private final int period;
    private final int loggingInterval;
    private int logInterval;
    private @NotNull ThreadSignal signal = ThreadSignal.PAUSED;
    public String subsystemName;
    Thread thisThread;

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
        this(period, Constants.DEFAULT_PERIODS_PER_LOG);
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
        if (thisThread == null || !thisThread.isAlive()) {
            thisThread = new Thread(this);
            thisThread.start();
        }
    }

    /**
     * This function will be called repeatedly when the thread is alive. The period will be whatever you defined when creating the
     * object
     */
    public void update() {

    }

    private final Map<String, Object> logDataMap = new HashMap<>();


    public void logData(String key, Object value) {
        //SmartDashboard.putString(key, value.toString());
        logDataMap.put(key, value);
    }

    int lastLength = 20;


    public void pushLog() {
        for (Map.Entry<String, Object> entry : logDataMap.entrySet()) {
            Object obj = entry.getValue();
            Class<?> cl = obj.getClass();

            //@formatter:off
            if (cl.equals(Integer.class)) SmartDashboard.putNumber(entry.getKey(), (int) obj);
            else if (cl.equals(Double.class)) SmartDashboard.putNumber(entry.getKey(), (double) obj);
            else if (cl.equals(Short.class)) SmartDashboard.putNumber(entry.getKey(), (short) obj);
            else if (cl.equals(Long.class)) SmartDashboard.putNumber(entry.getKey(), (long) obj);
            else if (cl.equals(Float.class)) SmartDashboard.putNumber(entry.getKey(), (float) obj);
            else if (cl.equals(Byte.class)) SmartDashboard.putNumber(entry.getKey(), (byte) obj);
            else if (cl.equals(Boolean.class)) SmartDashboard.putBoolean(entry.getKey(), (boolean) obj);
            else if (cl.equals(String.class)) SmartDashboard.putString(entry.getKey(), (String) obj);
            else if (cl.equals(Double[].class)) SmartDashboard.putNumberArray(entry.getKey(), (Double[]) obj);
            else if (cl.equals(Boolean[].class)) SmartDashboard.putBooleanArray(entry.getKey(), (Boolean[]) obj);
            else if (cl.equals(String[].class)) SmartDashboard.putStringArray(entry.getKey(), (String[]) obj);
            else SmartDashboard.putString(entry.getKey(), entry.getValue().toString());
            //@formatter:on
        }
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
                    pushLog();
                    
                    logInterval = 1;
                }
            }
            double executionTimeMS = (Timer.getFPGATimestamp() - startTime) * 1000;
            try {
                if (period - executionTimeMS > 0) {
                    Thread.sleep((long) (period - executionTimeMS));
                }
            } catch (Exception e) {
                System.out.println("Thread interrupted " + subsystemName + " message: " + e.getMessage());
                return;
            }

        }
    }
}
