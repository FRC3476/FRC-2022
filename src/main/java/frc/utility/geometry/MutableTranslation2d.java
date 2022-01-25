package frc.utility.geometry;

import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolatable;

import java.util.Objects;

/**
 * Represents a translation in 2d space. This object can be used to represent a point or a vector.
 *
 * <p>This assumes that you are using conventional mathematical axes. When the robot is placed on
 * the origin, facing toward the X direction, moving forward increases the X, whereas moving to the left increases the Y.
 */
@SuppressWarnings({"ParameterName", "MemberName"})
@JsonIgnoreProperties(ignoreUnknown = true)
@JsonAutoDetect(getterVisibility = JsonAutoDetect.Visibility.NONE)
public class MutableTranslation2d implements Interpolatable<edu.wpi.first.math.geometry.Translation2d> {
    private double m_x;
    private double m_y;

    /**
     * Constructs a Translation2d with X and Y components equal to zero.
     */
    public MutableTranslation2d() {
        this(0.0, 0.0);
    }

    /**
     * Constructs a new MutableTranslation2d from a Translation2d.
     */
    public MutableTranslation2d(Translation2d translation) {
        m_x = translation.getX();
        m_y = translation.getY();
    }

    public MutableTranslation2d(MutableTranslation2d translation) {
        this.m_x = translation.m_x;
        this.m_y = translation.m_y;
    }

    /**
     * Constructs a Translation2d with the X and Y components equal to the provided values.
     *
     * @param x The x component of the translation.
     * @param y The y component of the translation.
     */
    @JsonCreator
    public MutableTranslation2d(
            @JsonProperty(required = true, value = "x") double x,
            @JsonProperty(required = true, value = "y") double y) {
        m_x = x;
        m_y = y;
    }

    /**
     * Constructs a Translation2d with the provided distance and angle. This is essentially converting from polar coordinates to
     * Cartesian coordinates.
     *
     * @param distance The distance from the origin to the end of the translation.
     * @param angle    The angle between the x-axis and the translation vector.
     */
    public MutableTranslation2d(double distance, Rotation2d angle) {
        m_x = distance * angle.getCos();
        m_y = distance * angle.getSin();
    }

    /**
     * Sets this Translation2d to the X and Y components equal to the provided values.
     *
     * @param x The x component of the translation.
     * @param y The y component of the translation.
     */
    public MutableTranslation2d set(double x, double y) {
        m_x = x;
        m_y = y;
        return this;
    }


    /**
     * Set this Translation2d with the provided distance and angle. This is essentially converting from polar coordinates to
     * Cartesian coordinates.
     *
     * @param distance The distance from the origin to the end of the translation.
     * @param angle    The angle between the x-axis and the translation vector.
     */
    public MutableTranslation2d set(double distance, Rotation2d angle) {
        m_x = distance * angle.getCos();
        m_y = distance * angle.getSin();
        return this;
    }

    /**
     * Calculates the distance between two translations in 2d space.
     *
     * <p>This function uses the pythagorean theorem to calculate the distance. distance = sqrt((x2 -
     * x1)^2 + (y2 - y1)^2)
     *
     * @param other The translation to compute the distance to.
     * @return The distance between the two translations.
     */
    public double getDistance(edu.wpi.first.math.geometry.Translation2d other) {
        return Math.hypot(other.getX() - m_x, other.getY() - m_y);
    }

    /**
     * Returns the X component of the translation.
     *
     * @return The x component of the translation.
     */
    @JsonProperty
    public double getX() {
        return m_x;
    }

    /**
     * Returns the Y component of the translation.
     *
     * @return The y component of the translation.
     */
    @JsonProperty
    public double getY() {
        return m_y;
    }

    /**
     * Returns the norm, or distance from the origin to the translation.
     *
     * @return The norm of the translation.
     */
    public double getNorm() {
        return Math.hypot(m_x, m_y);
    }

    /**
     * Applies a rotation to the translation in 2d space.
     *
     * <p>This multiplies the translation vector by a counterclockwise rotation matrix of the given
     * angle. [x_new] [other.cos, -other.sin][x] [y_new] = [other.sin, other.cos][y]
     *
     * <p>For example, rotating a Translation2d of {2, 0} by 90 degrees will return a Translation2d of
     * {0, 2}.
     *
     * @param other The rotation to rotate the translation by.
     * @return The rotated translation.
     */
    public MutableTranslation2d rotateBy(Rotation2d other) {
        return set(m_x * other.getCos() - m_y * other.getSin(), m_x * other.getSin() + m_y * other.getCos());
    }

    /**
     * Adds two translations in 2d space and returns the sum. This is similar to vector addition.
     *
     * <p>For example, Translation2d{1.0, 2.5} + Translation2d{2.0, 5.5} = Translation2d{3.0, 8.0}
     *
     * @param other The translation to add.
     * @return The sum of the translations.
     */
    public MutableTranslation2d plus(edu.wpi.first.math.geometry.Translation2d other) {
        return set(m_x + other.getX(), m_y + other.getY());
    }

    /**
     * Adds two translations in 2d space and returns the sum. This is similar to vector addition.
     *
     * <p>For example, Translation2d{1.0, 2.5} + Translation2d{2.0, 5.5} = Translation2d{3.0, 8.0}
     *
     * @param other The translation to add.
     * @return The sum of the translations.
     */
    public MutableTranslation2d plus(MutableTranslation2d other) {
        return set(m_x + other.getX(), m_y + other.getY());
    }

    /**
     * Subtracts the other translation from the other translation and returns the difference.
     *
     * <p>For example, Translation2d{5.0, 4.0} - Translation2d{1.0, 2.0} = Translation2d{4.0, 2.0}
     *
     * @param other The translation to subtract.
     * @return The difference between the two translations.
     */
    public MutableTranslation2d minus(edu.wpi.first.math.geometry.Translation2d other) {
        return set(m_x - other.getX(), m_y - other.getY());
    }

    /**
     * Returns the inverse of the current translation. This is equivalent to rotating by 180 degrees, flipping the point over both
     * axes, or simply negating both components of the translation.
     *
     * @return The inverse of the current translation.
     */
    public edu.wpi.first.math.geometry.Translation2d unaryMinus() {
        return new edu.wpi.first.math.geometry.Translation2d(-m_x, -m_y);
    }

    /**
     * Multiplies the translation by a scalar and returns the new translation.
     *
     * <p>For example, Translation2d{2.0, 2.5} * 2 = Translation2d{4.0, 5.0}
     *
     * @param scalar The scalar to multiply by.
     * @return The scaled translation.
     */
    public MutableTranslation2d times(double scalar) {
        return set(m_x * scalar, m_y * scalar);
    }

    /**
     * Divides the translation by a scalar and returns the new translation.
     *
     * <p>For example, Translation2d{2.0, 2.5} / 2 = Translation2d{1.0, 1.25}
     *
     * @param scalar The scalar to multiply by.
     * @return This mutated object.
     */
    public MutableTranslation2d div(double scalar) {
        return set(m_x / scalar, m_y / scalar);
    }

    @Override
    public String toString() {
        return String.format("Translation2d(X: %.2f, Y: %.2f)", m_x, m_y);
    }

    /**
     * Checks equality between this Translation2d and another object.
     *
     * @param obj The other object.
     * @return Whether the two objects are equal or not.
     */
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof edu.wpi.first.math.geometry.Translation2d) {
            return Math.abs(((edu.wpi.first.math.geometry.Translation2d) obj).getX() - m_x) < 1E-9
                    && Math.abs(((edu.wpi.first.math.geometry.Translation2d) obj).getY() - m_y) < 1E-9;
        } else if (obj instanceof MutableTranslation2d) {
            return Math.abs(((MutableTranslation2d) obj).getX() - m_x) < 1E-9
                    && Math.abs(((MutableTranslation2d) obj).getY() - m_y) < 1E-9;
        }
        return false;
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_x, m_y);
    }

    @Override
    public edu.wpi.first.math.geometry.Translation2d interpolate(edu.wpi.first.math.geometry.Translation2d endValue, double t) {
        return new edu.wpi.first.math.geometry.Translation2d(
                MathUtil.interpolate(this.getX(), endValue.getX(), t),
                MathUtil.interpolate(this.getY(), endValue.getY(), t));
    }

    public Translation2d getTranslation2d() {
        return new Translation2d(m_x, m_y);
    }
}
