package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

public class ShootAndMoveLow extends AbstractGuiAuto {
    public ShootAndMoveLow() {
        super(Filesystem.getDeployDirectory().getPath() + "/shooter/shootandmovelow.json");
    }
}
