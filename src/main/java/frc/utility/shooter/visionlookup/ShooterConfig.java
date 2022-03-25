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

    @Override
    public String toString() {
        return "ShooterConfig{" +
                "shooterConfigs=" + shooterConfigs +
                '}';
    }

    public void printCSV() {
        StringBuilder sb = new StringBuilder();
        for (ShooterPreset shooterPreset : shooterConfigs) {
            sb.append(shooterPreset.getDistance()).append(",")
                    .append(shooterPreset.getFlywheelSpeed()).append(",")
                    .append(shooterPreset.getHoodEjectAngle()).append("\n");
        }
        System.out.println(sb);
    }
}
