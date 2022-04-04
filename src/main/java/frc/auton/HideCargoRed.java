package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class HideCargoRed extends AbstractGuiAuto {
    public HideCargoRed() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/red/hidecargo.json"));
    }
}
