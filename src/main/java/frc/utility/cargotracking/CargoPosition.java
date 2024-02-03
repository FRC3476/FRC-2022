package frc.utility.cargotracking;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.joml.Vector3d;

public class CargoPosition {

    public @NotNull Vector3d position;
    public @Nullable Vector3d velocity;
    public double lastUpdateTime;
    public double error;
    public int framesTillDeletion = -1;

    public CargoPosition(@NotNull Vector3d position, @Nullable Vector3d velocity, double lastUpdateTime) {
        this.position = position;
        this.velocity = velocity;
        this.lastUpdateTime = lastUpdateTime;
    }
}
