package frc.utility.geometry;

import edu.wpi.first.math.geometry.Pose2d;

public final class Pose2dUtils {
    public static Pose2d multiply(Pose2d pose2d, double scalar) {
        return new Pose2d(pose2d.getTranslation().times(scalar), pose2d.getRotation().times(scalar));
    }
}
