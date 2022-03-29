package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class BuddyAutoLeftHideBlue extends AbstractGuiAuto {
    public BuddyAutoLeftHideBlue() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/blue/buddyautoleftsteal.json"));
    }
}
