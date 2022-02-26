package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class FourBall extends AbstractGuiAuto {
    public FourBall() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/4ball.json"));
    }
}
