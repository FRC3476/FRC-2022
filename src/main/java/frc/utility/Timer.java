package frc.utility;

public final class Timer {

    static boolean manuallySetTime = false;
    static double time = 0;

    public static void setTime(double sesTime) {
        manuallySetTime = true;
        time = sesTime;
    }

    public static double getFPGATimestamp() {
        if (manuallySetTime) return time;
        return edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    }
}
