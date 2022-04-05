package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class InsideClimbRed extends AbstractGuiAuto {
    public InsideClimbRed() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/red/insideClimb.json"));
    }
}
