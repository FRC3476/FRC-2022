package frc.auton;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

public class ShootAndMoveHigh extends AbstractGuiAuto {
    public ShootAndMoveHigh(){
        super(Filesystem.getDeployDirectory().getPath() + "/shooter/shootandmovehigh.json");
    }
}
