package frc.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.utility.Timer;
import frc.utility.controllers.LazyTalonSRX;

import java.util.function.Consumer;
import java.util.function.Function;

public class Climber extends AbstractSubsystem {
    private static Climber instance = new Climber();

    private final LazyTalonSRX climberMotor;
    private final LazyTalonSRX climberMotor2;

    private final DigitalInput elevatorArmContactSwitchA;
    private final DigitalInput elevatorArmContactSwitchB;

    private final DigitalInput pivotingArmContactSwitchA;
    private final DigitalInput pivotingArmContactSwitchB;
    private final DigitalInput pivotingArmLatchedSwitchA;
    private final DigitalInput pivotingArmLatchedSwitchB;

    private final Solenoid latchSolenoid;
    private final Solenoid pivotSolenoid;

    Thread climbThread;

    double data;

    enum ClimbState {
        LOWER_ELEVATOR_ARM_TILL_PIVOT_ARM_CONTACT(
                (Climber cl) -> cl.climberMotor.set(ControlMode.PercentOutput, -1),
                (Climber cl) -> cl.elevatorArmContactSwitchA.get() && cl.elevatorArmContactSwitchB.get(),
                (Climber cl) -> cl.climberMotor.set(ControlMode.PercentOutput, 0)
        ),

        LATCH_PIVOT_ARM(
                (Climber cl) -> cl.latchSolenoid.set(true),
                (Climber cl) -> cl.pivotingArmLatchedSwitchA.get() && cl.pivotingArmLatchedSwitchB.get(),
                (Climber cl) -> {}
        ),

        MOVE_WEIGHT_TO_PIVOT_ARM(
                (Climber cl) -> {
                    cl.data = cl.climberMotor.getSelectedSensorPosition(0) + 100; //TODO: config this
                    cl.climberMotor.set(ControlMode.Position, cl.data);
                },
                (Climber cl) -> cl.climberMotor.getSelectedSensorPosition(0) > cl.data,
                (Climber cl) -> cl.climberMotor.set(ControlMode.PercentOutput, 0)
        ),

        PIVOT_PIVOT_ARM(
                (Climber cl) -> {
                    cl.pivotSolenoid.set(true);
                    cl.data = Timer.getFPGATimestamp();
                },
                (Climber cl) -> Timer.getFPGATimestamp() - cl.data > 0.5, //TODO config this time
                (Climber cl) -> {}
        ),

        EXTEND_ELEVATOR_ARM_TO_MAX_SAFE_LENGTH(
                (Climber cl) -> {
                    cl.data = cl.climberMotor.getSelectedSensorPosition(0) + 1000; //TODO: config this
                    cl.climberMotor.set(ControlMode.Position, cl.data);
                },
                (Climber cl) -> cl.climberMotor.getSelectedSensorPosition(0) > cl.data,
                (Climber cl) -> cl.climberMotor.set(ControlMode.PercentOutput, 0)

        ),

        WAIT_TILL_EXTENSION_IS_SAFE(
                (Climber cl) -> {},
                (Climber cl) -> {
                    AHRS gyro = RobotTracker.getInstance().getGyro();
                    return gyro.getPitch() < 0 && gyro.getRawAccelY() < 0;
                },
                (Climber cl) -> {}
        ),

        EXTEND_ELEVATOR_ARM_PAST_SAFE_LENGTH(
                (Climber cl) -> {
                    cl.data = cl.climberMotor.getSelectedSensorPosition(0) + 100; //TODO: config this
                    cl.climberMotor.set(ControlMode.Position, cl.data);
                },
                (Climber cl) -> cl.climberMotor.getSelectedSensorPosition(0) > cl.data,
                (Climber cl) -> cl.climberMotor.set(ControlMode.PercentOutput, 0)
        ),

        CONTACT_ELEVATOR_ARM_WITH_NEXT_BAR(
                (Climber cl) -> cl.climberMotor.set(ControlMode.PercentOutput, -1),
                (Climber cl) -> cl.elevatorArmContactSwitchA.get() && cl.elevatorArmContactSwitchB.get(),
                (Climber cl) -> cl.climberMotor.set(ControlMode.PercentOutput, 0)
        ),

        UNLATCH_PIVOT_ARM(
                (Climber cl) -> {
                    cl.latchSolenoid.set(false);
                    cl.data = Timer.getFPGATimestamp();
                },
                (Climber cl) -> Timer.getFPGATimestamp() - cl.data > 0.5 && !cl.pivotingArmContactSwitchA.get() && !cl.pivotingArmContactSwitchB.get(),
                (Climber cl) -> {}
        ),

        UNPIVOT_PIVOT_ARM(
                (Climber cl) -> {
                    cl.pivotSolenoid.set(false);
                    cl.data = Timer.getFPGATimestamp();
                },
                (Climber cl) -> Timer.getFPGATimestamp() - cl.data > 0.5,
                (Climber cl) -> {}
        );


        ClimbState(Consumer<Climber> startAction, Function<Climber, Boolean> waitCondition, Consumer<Climber> endAction) {
            this.startAction = startAction;
            this.waitCondition = waitCondition;
            this.endAction = endAction;
        }

        final Consumer<Climber> startAction;
        final Function<Climber, Boolean> waitCondition;
        final Consumer<Climber> endAction;
    }


    private Climber() {
        super(Constants.CLIMBER_PERIOD, 5);

        climberMotor = new LazyTalonSRX(Constants.CLIMBER_MOTOR_ID);
        climberMotor2 = new LazyTalonSRX(Constants.CLIMBER_MOTOR_2_ID);
        climberMotor2.follow(climberMotor);
        climberMotor.config_kF(0, Constants.CLIMBER_MOTOR_KF);
        climberMotor.config_kP(0, Constants.CLIMBER_MOTOR_KP);
        climberMotor.config_kI(0, Constants.CLIMBER_MOTOR_KI);
        climberMotor.config_kD(0, Constants.CLIMBER_MOTOR_KD);
        climberMotor.config_IntegralZone(0, Constants.CLIMBER_MOTOR_IZONE);
        climberMotor.configMaxIntegralAccumulator(0, Constants.CLIMBER_MOTOR_MAX_IACCUMULATOR);
        climberMotor.configPeakOutputForward(Constants.CLIMBER_MOTOR_MAX_OUTPUT);
        climberMotor.configPeakOutputReverse(Constants.CLIMBER_MOTOR_MAX_OUTPUT);

        elevatorArmContactSwitchA = new DigitalInput(Constants.ELEVATOR_ARM_CONTACT_SWITCH_A_DIO_CHANNEL);
        elevatorArmContactSwitchB = new DigitalInput(Constants.ELEVATOR_ARM_CONTACT_SWITCH_B_DIO_CHANNEL);

        pivotingArmContactSwitchA = new DigitalInput(Constants.PIVOTING_ARM_CONTACT_SWITCH_A_DIO_CHANNEL);
        pivotingArmContactSwitchB = new DigitalInput(Constants.PIVOTING_ARM_CONTACT_SWITCH_B_DIO_CHANNEL);
        pivotingArmLatchedSwitchA = new DigitalInput(Constants.PIVOTING_ARM_LATCHED_SWITCH_A_DIO_CHANNEL);
        pivotingArmLatchedSwitchB = new DigitalInput(Constants.PIVOTING_ARM_LATCHED_SWITCH_B_DIO_CHANNEL);

        latchSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.LATCH_SOLENOID_ID); //TODO config solenoid type.
        pivotSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.PIVOT_SOLENOID_ID); //TODO config solenoid type.
    }

    public void startClimb() {

    }

    public void deployClimb() {
        climberMotor.set(ControlMode.Position, 1000); // TODO: Change position
    }

    @Override
    public void update() {

    }

    public void stopClimb() {
        climbThread.interrupt();
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
