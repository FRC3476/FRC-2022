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

    private double data;

    ClimbState climbState = ClimbState.IDLE;
    private boolean isPaused = false;
    private double pausedClimberSetpoint;
    private ControlMode pausedClimberMode;

    enum ClimbState {
        IDLE((cl) -> {}, (cl) -> false, (cl) -> {}),

        /**
         * Will immediately transition to the next state to start the climb sequence.
         */
        START_CLIMB((cl) -> {}, (cl) -> true, (cl) -> {}),

        /**
         * Lowers the elevator arm until the contact switch is pressed on the bar.
         */
        LOWER_ELEVATOR_ARM_TILL_PIVOT_ARM_CONTACT(
                (cl) -> cl.climberMotor.set(ControlMode.PercentOutput, -1),
                (cl) -> cl.pivotingArmContactSwitchA.get() && cl.pivotingArmContactSwitchB.get(),
                (cl) -> cl.stopClimberMotor()
        ),

        /**
         * Extends the solenoid to latch the pivoting arm onto the bar. Waits until the latch switch is pressed.
         */
        LATCH_PIVOT_ARM(
                (cl) -> cl.latchSolenoid.set(true),
                (cl) -> cl.pivotingArmLatchedSwitchA.get() && cl.pivotingArmLatchedSwitchB.get(),
                (cl) -> {}
        ),

        /**
         * Moves the elevator arm up to the maximum safe height. Will continue to the next state once the elevator is no longer
         * supporting the robot.
         */
        MOVE_WEIGHT_TO_PIVOT_ARM(
                (cl) -> {
                    //TODO: config these
                    cl.data = cl.climberMotor.getSelectedSensorPosition(0) + 100; //position when it is considered "unlatched"
                    cl.climberMotor.set(ControlMode.Position, cl.climberMotor.getSelectedSensorPosition(0) + 1000);
                },
                (cl) -> cl.climberMotor.getSelectedSensorPosition(0) > cl.data - Constants.CLIMBER_MOTOR_MAX_ERROR,
                (cl) -> {}
        ),

        /**
         * Extends the solenoid to pivot the robot and cause it to start swinging. Waits a minimum time or until the climber motor
         * has fully extended to it's maximum safe position.
         */
        PIVOT_PIVOT_ARM(
                (cl) -> {
                    cl.pivotSolenoid.set(true);
                    cl.data = Timer.getFPGATimestamp();
                },
                //TODO: config this time
                (cl) -> Timer.getFPGATimestamp() - cl.data > 0.5 && cl.climberMotor.getClosedLoopError() < Constants.CLIMBER_MOTOR_MAX_ERROR,
                (cl) -> {}
        ),

        /**
         * Uses the gyro to wait until the robot is swinging towards the field
         */
        WAIT_TILL_EXTENSION_IS_SAFE(
                (cl) -> {},
                (cl) -> {
                    AHRS gyro = RobotTracker.getInstance().getGyro();
                    return gyro.getPitch() < 0 && gyro.getWorldLinearAccelY() < 0; // TODO: tune these values
                },
                (cl) -> {}
        ),

        /**
         * Extends the elevator arm past the safe height. At this point the arm will contact the bar when the robot swings back
         * towards the bars.
         */
        EXTEND_ELEVATOR_ARM_PAST_SAFE_LENGTH(
                (cl) -> {
                    cl.data = cl.climberMotor.getSelectedSensorPosition(0) + 100; //TODO: config this
                    cl.climberMotor.set(ControlMode.Position, cl.data);
                },
                (cl) -> cl.climberMotor.getSelectedSensorPosition(0) > cl.data - Constants.CLIMBER_MOTOR_MAX_ERROR,
                (cl) -> {}
        ),

        /**
         * Waits for the gyro to report that the robot has stopped swinging.
         */
        WAIT_FOR_SWING_STOP(
                (cl) -> {},
                (cl) -> {
                    AHRS gyro = RobotTracker.getInstance().getGyro();
                    return gyro.getPitch() < 0 && Math.abs(gyro.getWorldLinearAccelY()) < 0.1; //TODO: Tune these values
                },
                (cl) -> {}
        ),

        /**
         * Retracts the elevator arm to until it contacts the bar.
         */
        CONTACT_ELEVATOR_ARM_WITH_NEXT_BAR(
                (cl) -> cl.climberMotor.set(ControlMode.PercentOutput, -1),
                (cl) -> cl.elevatorArmContactSwitchA.get() && cl.elevatorArmContactSwitchB.get(),
                (cl) -> cl.stopClimberMotor()
        ),

        /**
         * Unlatches the pivot arm so that the robot is supported by the elevator arm.
         */
        UNLATCH_PIVOT_ARM(
                (cl) -> {
                    cl.latchSolenoid.set(false);
                    cl.data = Timer.getFPGATimestamp();
                },
                //TODO: Change the time
                (cl) -> Timer.getFPGATimestamp() - cl.data > 0.5 && !cl.pivotingArmContactSwitchA.get() && !cl.pivotingArmContactSwitchB.get(),
                (cl) -> {}
        ),

        /**
         * Unpivot the elevator arm so that the pivot arm is now underneath the next bar.
         */
        UNPIVOT_PIVOT_ARM(
                (cl) -> {
                    cl.pivotSolenoid.set(false);
                    cl.data = Timer.getFPGATimestamp();
                },
                //TODO: Change the time
                (cl) -> Timer.getFPGATimestamp() - cl.data > 0.5,
                (cl) -> {}
        );


        /**
         * @param startAction   The action to perform when the state is entered
         * @param waitCondition The condition to wait for before transitioning to the next state (true = continue, false = wait)
         * @param endAction     The action to perform when the state is exited
         */
        ClimbState(Consumer<Climber> startAction, Function<Climber, Boolean> waitCondition, Consumer<Climber> endAction) {
            this.startAction = startAction;
            this.waitCondition = waitCondition;
            this.endAction = endAction;
        }

        /**
         * The action to perform when the state is entered.
         */
        final Consumer<Climber> startAction;

        /**
         * The condition to wait for before transitioning to the next state.
         * <p>
         * Returns true when the condition is met. (When true is returned, the climber state should be transitioned to the next
         * state.) aka (true = continue, false = wait)
         */
        final Function<Climber, Boolean> waitCondition;

        /**
         * The action to perform when the state is exited.
         */
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
        climbState = ClimbState.START_CLIMB;
    }

    public void stopClimb() {
        climbState = ClimbState.IDLE;
        stopClimberMotor();
    }

    public void pauseClimb() {
        isPaused = true;
        pausedClimberMode = climberMotor.getControlMode();
        if (pausedClimberMode == ControlMode.PercentOutput) {
            pausedClimberSetpoint = climberMotor.getMotorOutputPercent();
        } else {
            pausedClimberSetpoint = climberMotor.getClosedLoopTarget();
        }
        stopClimberMotor();
    }

    private void stopClimberMotor() {
        climberMotor.set(ControlMode.Position, climberMotor.getSelectedSensorPosition());
    }

    public void resumeClimb() {
        isPaused = false;
        climberMotor.set(pausedClimberMode, pausedClimberSetpoint);
    }

    public void deployClimb() {
        climberMotor.set(ControlMode.Position, 1000); // TODO: Change position
    }

    @Override
    public void update() {
        if (!isPaused) {
            if (climbState.waitCondition.apply(this)) {
                climbState.endAction.accept(this);
                climbState = ClimbState.values()[(climbState.ordinal() + 1) % ClimbState.values().length];
                climbState.startAction.accept(this);
            }
        }
    }

    public void resetClimb() {

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
