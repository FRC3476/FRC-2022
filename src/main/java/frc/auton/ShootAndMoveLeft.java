package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class ShootAndMoveLeft extends AbstractGuiAuto {
    public ShootAndMoveLeft() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/shootandmoveleft.json"));
    }
}
