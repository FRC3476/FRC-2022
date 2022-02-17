package frc.utility;

import edu.wpi.first.wpilibj.PneumaticHub;

public final class Pneumatics {
    private static PneumaticHub PNEUMATIC_HUB = null;

    public synchronized static PneumaticHub getPneumaticsHub() {
        if (PNEUMATIC_HUB == null) {
            PNEUMATIC_HUB = new PneumaticHub(3);
            PNEUMATIC_HUB.enableCompressorDigital();
        }
        return PNEUMATIC_HUB;
    }
}
