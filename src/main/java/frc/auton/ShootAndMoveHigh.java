package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

import java.io.File;

public class ShootAndMoveHigh extends AbstractGuiAuto {
    public ShootAndMoveHigh(){
        super(new File(Filesystem.getDeployDirectory().getPath() + "/autos/shootandmovehigh.json"));
    }
}
