package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class OneBall extends AbstractGuiAuto {
    public OneBall() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/1ball.json"));
    }
}
