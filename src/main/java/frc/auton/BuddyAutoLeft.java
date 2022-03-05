package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class BuddyAutoLeft extends AbstractGuiAuto {
    public BuddyAutoLeft() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/buddyautoleft.json"));
    }
}
