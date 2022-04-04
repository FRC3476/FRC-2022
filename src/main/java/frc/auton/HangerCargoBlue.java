package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class HangerCargoBlue extends AbstractGuiAuto {
    public HangerCargoBlue() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/blue/movecargotohanger.json"));
    }
}
