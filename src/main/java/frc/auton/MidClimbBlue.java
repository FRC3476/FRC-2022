package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class MidClimbBlue extends AbstractGuiAuto {
    public MidClimbBlue() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/blue/midClimb.json"));
    }
}
