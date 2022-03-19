package frc.utility;

public final class Timer {

    static boolean manuallySetTime = false;
    static double time = 0;

    public static void setTime(double sesTime) {
        manuallySetTime = true;
        time = sesTime;
    }

    /**
     * Return the system clock time in seconds. Return the time from the FPGA hardware clock in seconds since the FPGA started.
     *
     * @return Robot running time in seconds.
     */
    public static double getFPGATimestamp() {
        if (manuallySetTime) return time;
        return edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    }
}
