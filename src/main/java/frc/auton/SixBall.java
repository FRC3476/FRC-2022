package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

public class SixBall extends AbstractGuiAuto {
    public SixBall() {
        super(Filesystem.getDeployDirectory().getPath() + "/shooter/6ball.json");
    }
}
