package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class SixBallBlue extends AbstractGuiAuto {
    public SixBallBlue() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/blue/6ball.json"));
    }
}
