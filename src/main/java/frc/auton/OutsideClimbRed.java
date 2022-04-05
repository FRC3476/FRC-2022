package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class OutsideClimbRed extends AbstractGuiAuto {
    public OutsideClimbRed() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/red/outsideClimb.json"));
    }
}
