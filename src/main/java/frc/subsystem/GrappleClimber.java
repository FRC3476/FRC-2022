package frc.subsystem;

import frc.robot.Constants;

public class GrappleClimber extends AbstractSubsystem {
    // Singleon Setup
    private GrappleClimber instance = new GrappleClimber();

    public GrappleClimber getInstance() {
        return instance;
    }

    private GrappleClimber() {
        super(Constants.GRAPPLE_CLIMBER_PERIOD, 1);
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
