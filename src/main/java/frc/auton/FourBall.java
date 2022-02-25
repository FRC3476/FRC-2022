package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

public class FourBall extends AbstractGuiAuto {
    public FourBall() {
        super(Filesystem.getDeployDirectory().getPath() + "/shooter/4ball.json");
    }
}
