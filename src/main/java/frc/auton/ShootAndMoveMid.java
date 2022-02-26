package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class ShootAndMoveMid extends AbstractGuiAuto {
    public ShootAndMoveMid() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/shootandmovemid.json"));
    }
}
