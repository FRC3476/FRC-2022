package frc.subsystem;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

@SuppressWarnings("unused")
public abstract class AbstractSubsystem implements Runnable, AutoCloseable {
    private final int period;
    private final int loggingInterval;
    private int logInterval;
    private @NotNull ThreadSignal signal = ThreadSignal.PAUSED;
    public final @NotNull String subsystemName;
    private @Nullable Thread thisThread;

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

    public void logData(@NotNull String key, @NotNull Object value) {
        DashboardHandler.getInstance().log(key, value);
    }

    public void logData(@NotNull String key, @NotNull Object value, boolean logToNetworkTables) {
        DashboardHandler.getInstance().log(key, value, logToNetworkTables);
    }

    public void pause() {
        signal = ThreadSignal.PAUSED;
    }

    public void kill() {
        signal = ThreadSignal.DEAD;
    }

    public void start() {
        signal = ThreadSignal.ALIVE;
        if ((thisThread == null || !thisThread.isAlive()) && this.period > 0) {
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
    int lastLength = 20;

    @Override
    @SuppressWarnings("BusyWait")
    public void run() {
        while (signal != ThreadSignal.DEAD) {
            double startTime = Timer.getFPGATimestamp();
            if (signal == ThreadSignal.ALIVE) {
                update();
                
                logInterval++;
                if (logInterval > loggingInterval) {
                    logData();
                    logInterval = 0;
                }
            }
            double executionTimeMS = (Timer.getFPGATimestamp() - startTime) * 1000;
            logData(subsystemName + " Execution Time", executionTimeMS);
            try {
                if (period - executionTimeMS > 0) {
                    Thread.sleep((long) (period - executionTimeMS));
                }
            } catch (InterruptedException e) {
                System.out.println("Thread interrupted " + subsystemName + " message: " + e.getMessage());
                return;
            }
            logData(subsystemName + " Pediod Length", (Timer.getFPGATimestamp() - startTime) * 1000);
        }
    }
}
