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

        climberMotor = new LazyTalonSRX(Constants.CLIMBER_MOTOR_ID);
        climberMotor2 = new LazyTalonSRX(Constants.CLIMBER_MOTOR_2_ID);

        elevatorArmContactSwitchA = new DigitalInput(Constants.ELEVATOR_ARM_CONTACT_SWITCH_A_DIO_CHANNEL);
        elevatorArmContactSwitchB = new DigitalInput(Constants.ELEVATOR_ARM_CONTACT_SWITCH_B_DIO_CHANNEL);

        pivotingArmContactSwitchA = new DigitalInput(Constants.PIVOTING_ARM_CONTACT_SWITCH_A_DIO_CHANNEL);
        pivotingArmContactSwitchB = new DigitalInput(Constants.PIVOTING_ARM_CONTACT_SWITCH_B_DIO_CHANNEL);
        pivotingArmLatchedSwitchA = new DigitalInput(Constants.PIVOTING_ARM_LATCHED_SWITCH_A_DIO_CHANNEL);
        pivotingArmLatchedSwitchB = new DigitalInput(Constants.PIVOTING_ARM_LATCHED_SWITCH_B_DIO_CHANNEL);
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
