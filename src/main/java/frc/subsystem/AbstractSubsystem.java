package frc.subsystem;

import edu.wpi.first.wpilibj.Timer;

@SuppressWarnings("unused")
public abstract class AbstractSubsystem implements Runnable, AutoCloseable {
    private final int period;
    private final int loggingInterval;
    private int logInterval;
    private ThreadSignal signal = ThreadSignal.PAUSED;
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