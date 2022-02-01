package frc.utility.shooter.visionlookup;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.utility.Serializer;

import java.io.File;
import java.io.IOException;
import java.util.Collections;

public final class VisionLookUpTable {
    ShooterConfig shooterConfig;

    private static final VisionLookUpTable vt = new VisionLookUpTable();

    public static VisionLookUpTable getInstance() {
        return vt;
    }

    private VisionLookUpTable() {

        try {
            File shooterDataFile = new File(Filesystem.getDeployDirectory().getPath() + "/shooter/shooterconfig.json");
            shooterConfig = (ShooterConfig) Serializer.deserializeFromFile(shooterDataFile, ShooterConfig.class);
            System.out.println("sucesfully loaded shooter config from the file");
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


        //Tidal good
        // lookUpTable.add(new ShooterPreset(47, 4000, 58));
        // lookUpTable.add(new ShooterPreset(44, 4000, 73));
        // lookUpTable.add(new ShooterPreset(45, 4000, 111));
        // lookUpTable.add(new ShooterPreset(44, 4000, 124));
        // lookUpTable.add(new ShooterPreset(33.5, 5400, 150));
        // lookUpTable.add(new ShooterPreset(35, 5400, 164));
        // lookUpTable.add(new ShooterPreset(36, 5600, 173));
        // lookUpTable.add(new ShooterPreset(36, 5700, 212));
        // lookUpTable.add(new ShooterPreset(36, 6000, 228));
        //lookUpTable.add(new ShooterPreset(38.5, 5700, 223));


        //Good Balls
        // lookUpTable.add(new ShooterPreset(44, 4000, 58));
        // lookUpTable.add(new ShooterPreset(41, 4000, 73));
        // lookUpTable.add(new ShooterPreset(42, 4000, 111));
        // lookUpTable.add(new ShooterPreset(41, 4000, 124));
        // lookUpTable.add(new ShooterPreset(30.5, 5400, 150));
        // lookUpTable.add(new ShooterPreset(33, 5600, 173));
        // lookUpTable.add(new ShooterPreset(33, 5700, 212));


        Collections.sort(shooterConfig.getShooterConfigs());
    }

    public ShooterPreset getShooterPreset(double distanceFromTarget) {

        if (distanceFromTarget <= shooterConfig.getShooterConfigs().get(0).getDistance()) {
            return shooterConfig.getShooterConfigs().get(0);
        }

        for (int i = 1; i < shooterConfig.getShooterConfigs().size(); i++) {
            double dist = shooterConfig.getShooterConfigs().get(i).getDistance();

            if (dist == distanceFromTarget) {
                return shooterConfig.getShooterConfigs().get(i);
            } else if (dist > distanceFromTarget) {

                double percentIn = (distanceFromTarget - shooterConfig.getShooterConfigs().get(
                        i - 1).getDistance()) / (shooterConfig.getShooterConfigs().get(
                        i).getDistance() - shooterConfig.getShooterConfigs().get(i - 1).getDistance());
                //System.out.println(percentIn + " " + (dist - lookUpTable.get(i-1).getDistance()) + " " + ( lookUpTable.get(i)
                // .getDistance() - lookUpTable.get(i-1).getDistance())+ " " + distanceFromTarget);

                return interpolateShooterPreset(shooterConfig.getShooterConfigs().get(i - 1),
                        shooterConfig.getShooterConfigs().get(i), percentIn);
            }
        }

        return shooterConfig.getShooterConfigs().get(shooterConfig.getShooterConfigs().size() - 1);
    }
    //interpolate() is startValue + (endValue - startValue) * fraction


    private ShooterPreset interpolateShooterPreset(ShooterPreset startValue, ShooterPreset endValue, double percentIn) {
        double flywheelSpeed =
                startValue.getFlywheelSpeed() + (endValue.getFlywheelSpeed() - startValue.getFlywheelSpeed()) * percentIn;
        double hoodPosition =
                startValue.getHoodEjectAngle() + (endValue.getHoodEjectAngle() - startValue.getHoodEjectAngle()) * percentIn;
        double distance = startValue.getDistance() + (endValue.getDistance() - startValue.getDistance()) * percentIn;

        return new ShooterPreset(hoodPosition, flywheelSpeed, distance);
    }

    /**
     * <b>MAKE SURE YOU SORT THE LIST BEFORE CALLING THIS FUNCTION</b>
     *
     * @param shooterConfig a sorted shooter config
     */
    public void setShooterConfig(ShooterConfig shooterConfig) {
        this.shooterConfig = shooterConfig;
    }
}

