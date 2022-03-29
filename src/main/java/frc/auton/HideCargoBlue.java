package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class HideCargoBlue extends AbstractGuiAuto {
    public HideCargoBlue() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/blue/hidecargo.json"));
    }
}
