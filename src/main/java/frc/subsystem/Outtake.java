package frc.subsystem;

import frc.robot.Constants;
import org.jetbrains.annotations.NotNull;

public class Outtake extends AbstractSubsystem {

    private Hopper hopper = Hopper.getInstance();

    private static @NotNull Outtake instance = new Outtake();

    public static Outtake getInstance() {
        return instance;
    }

    /**
     * Outtake can either be OFF or ON
     */
    public enum OuttakeState {
        OFF,
        ON
    }

    private OuttakeState outtakeState = OuttakeState.OFF;

    private Outtake() {
        super(Constants.OUTTAKE_PERIOD);
    }

    private void updateOuttakeState() {
        if (hopper.isBallDetected()) {
            outtakeState = OuttakeState.ON;
        } else {
            outtakeState = OuttakeState.OFF;
        }
    }

    @Override
    public void update() {
        updateOuttakeState();

        switch (outtakeState) {
            case OFF:
                break;
            case ON:
                break;
        }
    }

    @Override
    public void selfTest() {

    }

    @Override
    public void logData() {

    }

    @Override
    public void close() throws Exception {

    }
}
