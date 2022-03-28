package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class HideBallsBlue extends AbstractGuiAuto {
    public HideBallsBlue() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/blue/hideballs.json"));
    }
}
