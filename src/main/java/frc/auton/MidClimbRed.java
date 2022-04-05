package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class MidClimbRed extends AbstractGuiAuto {
    public MidClimbRed() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/red/midClimb.json"));
    }
}
