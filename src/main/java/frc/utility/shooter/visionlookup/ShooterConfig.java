package frc.utility.shooter.visionlookup;

import com.fasterxml.jackson.annotation.JsonCreator;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.List;

public class ShooterConfig {
    private final @NotNull List<ShooterPreset> shooterConfigs;

    @JsonCreator
    public ShooterConfig() {
        shooterConfigs = new ArrayList<>();
    }

    public ShooterConfig(@NotNull ArrayList<ShooterPreset> shooterConfigs) {
        this.shooterConfigs = shooterConfigs;
    }

    public @NotNull List<ShooterPreset> getShooterConfigs() {
        return shooterConfigs;
    }
}
