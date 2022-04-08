package frc.utility.tracking;

import edu.wpi.first.math.geometry.Pose2d;


public final class TimestampedPose {
    public final double timestamp;
    public final Pose2d pose;

    public TimestampedPose(double timestamp, Pose2d pose) {
        this.timestamp = timestamp;
        this.pose = pose;
    }
}
