package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class BuddyAutoLeftHideRed extends AbstractGuiAuto {
    public BuddyAutoLeftHideRed() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/red/buddyautoleftsteal.json"));
    }
}
