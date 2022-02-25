package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

public class ShootAndMoveMid extends AbstractGuiAuto {
    public ShootAndMoveMid() {
        super(Filesystem.getDeployDirectory().getPath() + "/shooter/shootandmovemid.json");
    }
}
