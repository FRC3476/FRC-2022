package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class FiveBallRed extends AbstractGuiAuto {
    public FiveBallRed() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/red/5ball.json"));
    }
}
