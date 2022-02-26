package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class ShootAndMoveLow extends AbstractGuiAuto {
    public ShootAndMoveLow() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/shootandmovelow.json"));
    }
}
