package frc.utility.shooter.visionlookup;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.utility.Serializer;
import org.jetbrains.annotations.NotNull;

import java.io.File;
import java.io.IOException;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;

public final class VisionLookUpTable {

    private final ReentrantLock shooterConfigLock = new ReentrantLock();
    ShooterConfig shooterConfig;

    private static final VisionLookUpTable vt = new VisionLookUpTable();

    public static VisionLookUpTable getInstance() {
        return vt;
    }

    private VisionLookUpTable() {

        try {
            File shooterDataFile = new File(Filesystem.getDeployDirectory().getPath() + "/shooter/shooterconfig.json");
            shooterConfig = (ShooterConfig) Serializer.deserializeFromFile(shooterDataFile, ShooterConfig.class);
            System.out.println("Successfully loaded shooter config from the file");
        } catch (IOException e) {
            DriverStation.reportError("Failed to load shooter config from /shooter/shooterconfig.json. Using default values",
                    e.getStackTrace());
            shooterConfig = new ShooterConfig();
            shooterConfig.getShooterConfigs().add(new ShooterPreset(47, 4800, 58));
            shooterConfig.getShooterConfigs().add(new ShooterPreset(46, 4800, 73));
            shooterConfig.getShooterConfigs().add(new ShooterPreset(44, 5100, 111));
            shooterConfig.getShooterConfigs().add(new ShooterPreset(41, 5200, 124));
            shooterConfig.getShooterConfigs().add(new ShooterPreset(39, 5400, 150));
            shooterConfig.getShooterConfigs().add(new ShooterPreset(36.8, 5400, 164));
            shooterConfig.getShooterConfigs().add(new ShooterPreset(37, 5600, 173));
            shooterConfig.getShooterConfigs().add(new ShooterPreset(36, 5700, 212));
            shooterConfig.getShooterConfigs().add(new ShooterPreset(36, 6000, 228));
        }

        Collections.sort(shooterConfig.getShooterConfigs());
    }

    final @NotNull Comparator comparator = (o1, o2) -> {
        ShooterPreset sp = (ShooterPreset) o1;
        double d = (double) o2;
        return Double.compare(sp.getDistance(), d);
    };

    private static final ShooterPreset DEFAULT_PRESET = new ShooterPreset(90, 0, 0);

    public @NotNull ShooterPreset getShooterPreset(double distanceFromTarget) {
        List<ShooterPreset> sortedShooterConfigs;
        shooterConfigLock.lock();
        try {
            sortedShooterConfigs = shooterConfig.getShooterConfigs();
        } finally {
            shooterConfigLock.unlock();
        }

        int index = Collections.binarySearch(sortedShooterConfigs, new ShooterPreset(0, 0, distanceFromTarget));
        if (index < 0) { //Convert the binary search index into an actual index
            index = -(index + 1);
        }
        ShooterPreset interpolatedShooterPreset = DEFAULT_PRESET;
        double percentIn;
        if (!sortedShooterConfigs.isEmpty()) {
            if (sortedShooterConfigs.get(0).getDistance() >= distanceFromTarget) {
                interpolatedShooterPreset = sortedShooterConfigs.get(0);
            } else if (sortedShooterConfigs.get(sortedShooterConfigs.size() - 1).getDistance() < distanceFromTarget) {
                interpolatedShooterPreset = sortedShooterConfigs.get(sortedShooterConfigs.size() - 1);
            } else {
                //One of the above 2 if statements will true if there is only 1 element in the list
                percentIn = (distanceFromTarget - sortedShooterConfigs.get(index - 1).getDistance()) /
                        (sortedShooterConfigs.get(index).getDistance() - sortedShooterConfigs.get(index - 1).getDistance());
                interpolatedShooterPreset = interpolateShooterPreset(sortedShooterConfigs.get(index - 1),
                        sortedShooterConfigs.get(index), percentIn);
            }
        }
        return interpolatedShooterPreset;
    }


    private ShooterPreset interpolateShooterPreset(ShooterPreset startValue, ShooterPreset endValue, double percentIn) {
        //@formatter:off
        double flywheelSpeed = startValue.getFlywheelSpeed() + (endValue.getFlywheelSpeed() - startValue.getFlywheelSpeed()) * percentIn;
        double hoodPosition = startValue.getHoodEjectAngle() + (endValue.getHoodEjectAngle() - startValue.getHoodEjectAngle()) * percentIn;
        double distance = startValue.getDistance() + (endValue.getDistance() - startValue.getDistance()) * percentIn;
        //@formatter:on

        return new ShooterPreset(hoodPosition, flywheelSpeed, distance);
    }

    /**
     * @param shooterConfig a shooter config to use
     */
    public void setShooterConfig(ShooterConfig shooterConfig) {
        System.out.println("Loading a new shooter config");
        Collections.sort(shooterConfig.getShooterConfigs());
        shooterConfigLock.lock();
        try {
            this.shooterConfig = shooterConfig;
        } finally {
            shooterConfigLock.unlock();
        }
    }

    public void printShooterConfig() {
        shooterConfigLock.lock();
        try {
            shooterConfig.printCSV();
        } finally {
            shooterConfigLock.unlock();
        }
    }
}

