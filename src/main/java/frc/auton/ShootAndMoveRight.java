package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class ShootAndMoveRight extends AbstractGuiAuto {
    public ShootAndMoveRight() {
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/shootandmoveright.json"));
    }
}
