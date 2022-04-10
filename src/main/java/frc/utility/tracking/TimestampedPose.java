package frc.utility.tracking;

import edu.wpi.first.math.geometry.Pose2d;
import org.jetbrains.annotations.NotNull;


public final class TimestampedPose implements Comparable<TimestampedPose> {
    public final double timestamp;
    public final Pose2d pose;

    public TimestampedPose(double timestamp, Pose2d pose) {
        this.timestamp = timestamp;
        this.pose = pose;
    }


    @Override
    public int compareTo(@NotNull TimestampedPose o) {
        return Double.compare(timestamp, o.timestamp);
    }
}
