package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class FourBallBlue extends AbstractGuiAuto {
    public FourBallBlue() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/blue/4ball.json"));
    }
}
