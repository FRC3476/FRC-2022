package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class FiveBall extends AbstractGuiAuto {
    public FiveBall() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/5ball.json"));
    }
}
