package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class OutsideClimbBlue extends AbstractGuiAuto {
    public OutsideClimbBlue() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/blue/outsideClimb.json"));
    }
}
