package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class FiveBallBlue extends AbstractGuiAuto {
    public FiveBallBlue() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/blue/5ball.json"));
    }
}
