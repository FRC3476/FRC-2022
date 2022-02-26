package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class SixBall extends AbstractGuiAuto {
    public SixBall() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/6ball.json"));
    }
}
