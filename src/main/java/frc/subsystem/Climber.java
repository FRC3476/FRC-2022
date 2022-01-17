package frc.subsystem;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.utility.controllers.LazyTalonSRX;

public class Climber extends AbstractSubsystem {
    private static Climber instance = new Climber();

    LazyTalonSRX climberMotor;
    LazyTalonSRX climberMotor2;

    DigitalInput elevatorArmContactSwitchA;
    DigitalInput elevatorArmContactSwitchB;

    DigitalInput pivotingArmContactSwitchA;
    DigitalInput pivotingArmContactSwitchB;
    DigitalInput pivotingArmLatchedSwitchA;
    DigitalInput pivotingArmLatchedSwitchB;
    

    private Climber() {
        super(Constants.CLIMBER_PERIOD);
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
