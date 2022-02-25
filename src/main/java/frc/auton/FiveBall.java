package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

public class FiveBall extends AbstractGuiAuto {
    public FiveBall() {
        super(Filesystem.getDeployDirectory().getPath() + "/shooter/5ball.json");
    }
}
