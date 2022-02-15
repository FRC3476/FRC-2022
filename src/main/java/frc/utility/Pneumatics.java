package frc.utility;

import edu.wpi.first.wpilibj.PneumaticHub;

public final class Pneumatics {
    private static final PneumaticHub PNEUMATIC_HUB = new PneumaticHub(3);

    static {
        PNEUMATIC_HUB.enableCompressorDigital();
    }

    public static PneumaticHub getPneumaticsHub() {
        return PNEUMATIC_HUB;
    }
}
