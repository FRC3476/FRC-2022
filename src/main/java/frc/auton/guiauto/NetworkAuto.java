package frc.auton.guiauto;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.jetbrains.annotations.NotNull;

public class NetworkAuto extends AbstractGuiAuto {

    static final @NotNull NetworkTableInstance instance = NetworkTableInstance.getDefault();
    static final @NotNull NetworkTable table = instance.getTable("autodata");
    static final @NotNull NetworkTableEntry autoPath = table.getEntry("autoPath");

    public NetworkAuto() {
        super(autoPath.getString(null));
    }
}