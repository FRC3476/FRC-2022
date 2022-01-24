package frc.utility.geometry;

import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;

import java.util.Objects;

/**
 * A rotation in a 2d coordinate frame represented a point on the unit circle (cosine and sine).
 */
@JsonIgnoreProperties(ignoreUnknown = true)
@JsonAutoDetect(getterVisibility = JsonAutoDetect.Visibility.NONE)
public class MutableRotation2d implements Interpolatable<edu.wpi.first.math.geometry.Rotation2d> {
    private double value;
    private double cos;
    private double sin;

    /**
     * Constructs a Rotation2d with a default angle of 0 degrees.
     */
    public MutableRotation2d() {
        value = 0.0;
        cos = 1.0;
        sin = 0.0;
    }

    public MutableRotation2d(Rotation2d rotation) {
        this.value = rotation.getRadians();
        this.cos = rotation.getCos();
        this.sin = rotation.getSin();
    }

    public MutableRotation2d(MutableRotation2d translation) {
        this.value = translation.getRadians();
        cos = translation.getCos();
        sin = translation.getSin();
    }

    /**
     * Constructs a Rotation2d with the given radian value. The x and y don't have to be normalized.
     *
     * @param value The value of the angle in radians.
     */
    @JsonCreator
    public MutableRotation2d(@JsonProperty(required = true, value = "radians") double value) {
        this.value = value;
        cos = Math.cos(value);
        sin = Math.sin(value);
    }

    /**
     * Constructs a Rotation2d with the given x and y (cosine and sine) components.
     *
     * @param x The x component or cosine of the rotation.
     * @param y The y component or sine of the rotation.
     */
    public MutableRotation2d(double x, double y) {
        double magnitude = Math.hypot(x, y);
        if (magnitude > 1e-6) {
            sin = y / magnitude;
            cos = x / magnitude;
        } else {
            sin = 0.0;
            cos = 1.0;
        }
        value = Math.atan2(sin, cos);
    }

    /**
     * Constructs and returns a Rotation2d with the given degree value.
     *
     * @param degrees The value of the angle in degrees.
     * @return The rotation object with the desired angle value.
     */
    public static MutableRotation2d fromDegrees(double degrees) {
        return new MutableRotation2d(Math.toRadians(degrees));
    }

    /**
     * Adds two rotations together, with the result being bounded between -pi and pi.
     *
     * <p>For example, Rotation2d.fromDegrees(30) + Rotation2d.fromDegrees(60) = Rotation2d{-pi/2}
     *
     * @param other The rotation to add.
     * @return The sum of the two rotations.
     */
    public MutableRotation2d plus(MutableRotation2d other) {
        return rotateBy(other);
    }

    /**
     * Adds two rotations together, with the result being bounded between -pi and pi.
     *
     * <p>For example, Rotation2d.fromDegrees(30) + Rotation2d.fromDegrees(60) = Rotation2d{-pi/2}
     *
     * @param other The rotation to add.
     * @return The sum of the two rotations.
     */
    public MutableRotation2d plus(Rotation2d other) {
        return rotateBy(other);
    }

    /**
     * Subtracts the new rotation from the current rotation and returns the new rotation.
     *
     * <p>For example, Rotation2d.fromDegrees(10) - Rotation2d.fromDegrees(100) = Rotation2d{-pi/2}
     *
     * @param other The rotation to subtract.
     * @return The difference between the two rotations.
     */
    public MutableRotation2d minus(MutableRotation2d other) {
        return rotateBy(other.unaryMinus());
    }

    /**
     * Subtracts the new rotation from the current rotation and returns the new rotation.
     *
     * <p>For example, Rotation2d.fromDegrees(10) - Rotation2d.fromDegrees(100) = Rotation2d{-pi/2}
     *
     * @param other The rotation to subtract.
     * @return The difference between the two rotations.
     */
    public MutableRotation2d minus(Rotation2d other) {
        return rotateBy(other.unaryMinus());
    }

    /**
     * Takes the inverse of the current rotation. This is simply the negative of the current angular value.
     *
     * @return The inverse of the current rotation.
     */
    public MutableRotation2d unaryMinus() {
        return setRadians(-value);
    }

    /**
     * Multiplies the current rotation by a scalar.
     *
     * @param scalar The scalar.
     * @return This scaled Rotation2d.
     */
    public MutableRotation2d times(double scalar) {
        return setRadians(value * scalar);
    }

    /**
     * Adds the new rotation to the current rotation using a rotation matrix.
     *
     * <p>The matrix multiplication is as follows:
     *
     * <pre>
     * [cos_new]   [other.cos, -other.sin][cos]
     * [sin_new] = [other.sin,  other.cos][sin]
     * value_new = atan2(sin_new, cos_new)
     * </pre>
     *
     * @param other The rotation to rotate by.
     * @return This rotated Rotation2d.
     */
    public MutableRotation2d rotateBy(MutableRotation2d other) {
        return set(cos * other.cos - sin * other.sin, cos * other.sin + sin * other.cos);
    }

    /**
     * Adds the new rotation to the current rotation using a rotation matrix.
     *
     * <p>The matrix multiplication is as follows:
     *
     * <pre>
     * [cos_new]   [other.cos, -other.sin][cos]
     * [sin_new] = [other.sin,  other.cos][sin]
     * value_new = atan2(sin_new, cos_new)
     * </pre>
     *
     * @param other The rotation to rotate by.
     * @return This rotated Rotation2d.
     */
    public MutableRotation2d rotateBy(Rotation2d other) {
        return set(cos * other.getCos() - sin * other.getSin(), cos * other.getSin() + sin * other.getCos());
    }

    /**
     * Returns the radian value of the rotation.
     *
     * @return The radian value of the rotation.
     */
    @JsonProperty
    public double getRadians() {
        return value;
    }

    /**
     * Returns the degree value of the rotation.
     *
     * @return The degree value of the rotation.
     */
    public double getDegrees() {
        return Math.toDegrees(value);
    }

    /**
     * Returns the cosine of the rotation.
     *
     * @return The cosine of the rotation.
     */
    public double getCos() {
        return cos;
    }

    /**
     * Returns the sine of the rotation.
     *
     * @return The sine of the rotation.
     */
    public double getSin() {
        return sin;
    }

    /**
     * Returns the tangent of the rotation.
     *
     * @return The tangent of the rotation.
     */
    public double getTan() {
        return sin / cos;
    }

    /**
     * Sets this MutableRotation2d to the given radian value. The x and y don't have to be normalized.
     *
     * @param radians The value of the angle in radians.
     * @return This MutableRotation2d.
     */
    public MutableRotation2d setRadians(double radians) {
        this.value = radians;
        this.cos = Math.cos(value);
        this.sin = Math.sin(value);
        return this;
    }

    /**
     * Sets this MutableRotation2d to the given degree value.
     *
     * @param degrees The value of the angle in degrees.
     * @return This rotation with desired angle value.
     */
    public MutableRotation2d setDegrees(double degrees) {
        setRadians(Math.toRadians(degrees));
        return this;
    }

    /**
     * Sets this Rotation2d with the given x and y (cosine and sine) components.
     *
     * @param x The x component or cosine of the rotation.
     * @param y The y component or sine of the rotation.
     * @return This rotation with desired x and y components.
     */
    public MutableRotation2d set(double x, double y) {
        double magnitude = Math.hypot(x, y);
        if (magnitude > 1e-6) {
            sin = y / magnitude;
            cos = x / magnitude;
        } else {
            sin = 0.0;
            cos = 1.0;
        }
        value = Math.atan2(sin, cos);
        return this;
    }


    @Override
    public String toString() {
        return String.format("Rotation2d(Rads: %.2f, Deg: %.2f)", value, Math.toDegrees(value));
    }

    /**
     * Checks equality between this Rotation2d and another object.
     *
     * @param obj The other object.
     * @return Whether the two objects are equal or not.
     */
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof MutableRotation2d) {
            var other = (MutableRotation2d) obj;
            return Math.hypot(cos - other.cos, sin - other.sin) < 1E-9;
        } else if (obj instanceof Rotation2d) {
            var other = (Rotation2d) obj;
            return Math.hypot(cos - other.getCos(), sin - other.getSin()) < 1E-9;
        }
        return false;
    }

    @Override
    public int hashCode() {
        return Objects.hash(value);
    }

    @Override
    @SuppressWarnings("ParameterName")
    public edu.wpi.first.math.geometry.Rotation2d interpolate(edu.wpi.first.math.geometry.Rotation2d endValue, double t) {
        return new edu.wpi.first.math.geometry.Rotation2d(MathUtil.interpolate(this.getRadians(), endValue.getRadians(), t));
    }
}