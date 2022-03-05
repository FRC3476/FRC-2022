package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class BuddyAutoRight extends AbstractGuiAuto {
    public BuddyAutoRight() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/buddyautoright.json"));
    }
}
