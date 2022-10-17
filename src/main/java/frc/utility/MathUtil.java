package frc.utility;

import edu.wpi.first.math.geometry.Translation2d;
import frc.utility.geometry.MutableTranslation2d;
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

    /**
     * Returns the squared distance between two points.
     *
     * @param first  The first point.
     * @param second The second point.
     * @return The squared distance between the two points.
     */
    @SuppressWarnings("ParameterName")
    public static double dist2(@NotNull Translation2d first, @NotNull MutableTranslation2d second) {
        return Math.pow(first.getX() - second.getX(), 2) + Math.pow(first.getY() - second.getY(), 2);
    }

    /**
     * Returns the squared distance between two points.
     *
     * @param first  The first point.
     * @param second The second point.
     * @return The squared distance between the two points.
     */
    @SuppressWarnings("ParameterName")
    public static double dist2(@NotNull MutableTranslation2d first, @NotNull Translation2d second) {
        return Math.pow(first.getX() - second.getX(), 2) + Math.pow(first.getY() - second.getY(), 2);
    }

    /**
     * Returns the squared distance between two points.
     *
     * @param first  The first point.
     * @param second The second point.
     * @return The squared distance between the two points.
     */
    @SuppressWarnings("ParameterName")
    public static double dist2(@NotNull MutableTranslation2d first, @NotNull MutableTranslation2d second) {
        return Math.pow(first.getX() - second.getX(), 2) + Math.pow(first.getY() - second.getY(), 2);
    }

    public static double dist2(@NotNull Translation2d translation2d) {
        double x = translation2d.getX();
        double y = translation2d.getY();
        return x * x + y * y;
    }
    /**
     * Returns the output of the modulo operation, but always with a positive result.
     *
     * @param x The number to modulo.
     * @param y The modulus.
     * @return The output of the modulo operation.
     */
    public static double mod(double x, double y) {
        double result = x % y;
        if (result < 0) {
            result += y;
        }
        return result;
    }

    /**
     * Returns the minimum of the supplied values.
     *
     * @param values The values to compare.
     * @return The minimum of the values.
     */
    public static double min(double @NotNull ... values) {
        double min = Double.MAX_VALUE;
        for (double value : values) {
            min = Math.min(min, value);
        }
        return min;
    }

}
