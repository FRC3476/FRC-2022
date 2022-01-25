package frc.utility;

import edu.wpi.first.math.geometry.Translation2d;
import org.jetbrains.annotations.NotNull;

public final class MathUtil {
    /**
     * Returns the squared distance between two points.
     *
     * @param first  The first point.
     * @param second The second point.
     * @return The squared distance between the two points.
     */
    @SuppressWarnings("ParameterName")
    public static double dist2(@NotNull Translation2d first, @NotNull Translation2d second) {
        return Math.pow(first.getX() - second.getX(), 2) + Math.pow(first.getY() - second.getY(), 2);
    }
}
