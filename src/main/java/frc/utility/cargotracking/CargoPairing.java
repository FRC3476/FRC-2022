package frc.utility.cargotracking;

import org.jetbrains.annotations.NotNull;
import org.joml.Vector3d;

import java.util.Objects;

public class CargoPairing implements Comparable<CargoPairing> {
    public final @NotNull CargoPosition mappedCargoPosition;
    public final @NotNull Vector3d detectedCargoPosition;
    public double errorSq;

    public CargoPairing(@NotNull CargoPosition mappedCargoPosition, @NotNull Vector3d detectedCargoPosition) {
        this.mappedCargoPosition = mappedCargoPosition;
        this.detectedCargoPosition = detectedCargoPosition;
        this.errorSq = mappedCargoPosition.position.distanceSquared(detectedCargoPosition);
    }

    @Override
    public int compareTo(@NotNull CargoPairing o) {
        return Double.compare(errorSq, o.errorSq);
    }

    @Override
    public boolean equals(Object obj) {
        return obj instanceof CargoPairing && ((CargoPairing) obj).mappedCargoPosition.equals(mappedCargoPosition)
                && ((CargoPairing) obj).detectedCargoPosition.equals(detectedCargoPosition);
    }

    @Override
    public int hashCode() {
        return Objects.hash(mappedCargoPosition, detectedCargoPosition, errorSq);
    }
}
