package frc.auton.guiauto.serialization;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIgnore;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.math.geometry.Rotation2d;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

@JsonIgnoreProperties(ignoreUnknown = true)
public final class TimedRotation {
    @JsonProperty("time")
    public double time;

    @JsonProperty("rotation")
    public Rotation2d rotation;

    @JsonIgnore
    public TimedRotation() {
        this(0, new Rotation2d());
    }

    @JsonIgnore
    public TimedRotation(Rotation2d rotation) {
        this(0, rotation);
    }

    @JsonCreator
    public TimedRotation(@JsonProperty("time") double time,
                         @JsonProperty("rotation") Rotation2d rotation) {
        this.time = time;
        this.rotation = rotation;
    }

    @JsonIgnore
    public TimedRotation(@NotNull TimedRotation other) {
        this(other.time, other.rotation);
    }

    @JsonIgnore
    @Contract(value = " -> new", pure = true)
    @NotNull
    public TimedRotation copy() {
        return new TimedRotation(this);
    }

    @JsonIgnore
    public Rotation2d getRotation() {
        return rotation;
    }
}
