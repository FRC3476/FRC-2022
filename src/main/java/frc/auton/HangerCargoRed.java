package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class HangerCargoRed extends AbstractGuiAuto {
    public HangerCargoRed() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/red/movecargotohanger.json"));
    }
}
