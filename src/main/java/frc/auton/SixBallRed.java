package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class SixBallRed extends AbstractGuiAuto {
    public SixBallRed() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/red/6ball.json"));
    }
}
