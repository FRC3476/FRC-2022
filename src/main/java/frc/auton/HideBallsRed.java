package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class HideBallsRed extends AbstractGuiAuto {
    public HideBallsRed() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/red/hideballs.json"));
    }
}
