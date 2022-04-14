package frc.utility.tracking;

import edu.wpi.first.math.geometry.Pose2d;
import org.jetbrains.annotations.NotNull;

import java.util.Objects;


public final class TimestampedPose implements Comparable<TimestampedPose> {
    public final double timestamp;
    public final @NotNull Pose2d pose;

    public TimestampedPose(double timestamp, @NotNull Pose2d pose) {
        this.timestamp = timestamp;
        this.pose = pose;
    }


    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        TimestampedPose that = (TimestampedPose) obj;
        return Double.compare(that.timestamp, timestamp) == 0 && Objects.equals(pose, that.pose);
    }

    @Override
    public int hashCode() {
        return Objects.hash(timestamp, pose);
    }

    @Override
    public int compareTo(@NotNull TimestampedPose o) {
        return Double.compare(timestamp, o.timestamp);
    }
}
