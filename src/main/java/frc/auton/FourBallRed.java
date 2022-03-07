package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class FourBallRed extends AbstractGuiAuto {
    public FourBallRed() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/red/4ball.json"));
    }
}
