package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class InsideClimbBlue extends AbstractGuiAuto {
    public InsideClimbBlue() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/blue/insideClimb.json"));
    }
}
