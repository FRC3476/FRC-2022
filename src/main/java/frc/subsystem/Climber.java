package frc.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
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

    public static Climber getInstance() {
        return instance;
    }

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
    private final Solenoid brakeSolenoid;

    private double data;

    ClimbState climbState = ClimbState.IDLE;
    private boolean isPaused = false;
    private double pausedClimberSetpoint;
    private ControlMode pausedClimberMode;

    private boolean stepByStep = false;
    private boolean advanceStep = false;
    private boolean skipChecks = false;
    private boolean ranEndAction = false;

    private double gyroPitchVelocity = 0;
    private double lastGyroPitch;
    private double gyroRollVelocity = 0;
    private double lastGyroRoll;

    public enum LatchState {
        LATCHED, UNLATCHED
    }

    enum PivotState {
        PIVOTED, INLINE
    }

    public enum BrakeState {
        BRAKING, FREE
    }

    public void setBrakeState(BrakeState brakeState) {
        brakeSolenoid.set(brakeState == BrakeState.FREE);
    }

    public void setLatchState(LatchState latchState) {
        latchSolenoid.set(latchState == LatchState.UNLATCHED);
    }

    public void setPivotState(PivotState pivotState) {
        pivotSolenoid.set(pivotState == PivotState.PIVOTED);
    }

    public BrakeState getBrakeState() {
        return brakeSolenoid.get() ? BrakeState.BRAKING : BrakeState.FREE;
    }

    public LatchState getLatchState() {
        return latchSolenoid.get() ? LatchState.UNLATCHED : LatchState.LATCHED;
    }

    public PivotState getPivotState() {
        return pivotSolenoid.get() ? PivotState.PIVOTED : PivotState.INLINE;
    }

    public enum ClimbState {
        IDLE((cl) -> {}, (cl) -> false, (cl) -> {}),

        /**
         * Will immediately transition to the next state to start the climb sequence.
         */
        START_CLIMB((cl) -> {}, (cl) -> true, (cl) -> {}),

        /**
         * Lowers the elevator arm until the contact switch is pressed on the bar.
         */
        LOWER_ELEVATOR_ARM_TILL_PIVOT_ARM_CONTACT(
                (cl) -> {
                    cl.climberMotor.set(ControlMode.PercentOutput, -Constants.CLIMBER_MOTOR_MAX_OUTPUT);
                    cl.setLatchState(LatchState.UNLATCHED);
                },
                (cl) -> cl.pivotingArmContactSwitchA.get() && cl.pivotingArmContactSwitchB.get(),
                (cl) -> cl.stopClimberMotor()
        ),

        /**
         * Extends the solenoid to latch the pivoting arm onto the bar. Waits until the latch switch is pressed.
         */
        LATCH_PIVOT_ARM(
                (cl) -> cl.setLatchState(LatchState.LATCHED),
                (cl) -> cl.pivotingArmLatchedSwitchA.get() && cl.pivotingArmLatchedSwitchB.get(),
                (cl) -> {}
        ),

        /**
         * Moves the elevator arm up to the maximum safe height. Will continue to the next state once the elevator is no longer
         * supporting the robot.
         */
        MOVE_WEIGHT_TO_PIVOT_ARM(
                (cl) -> {
                    //position when it is considered "unlatched"
                    cl.data = cl.climberMotor.getSelectedSensorPosition() + Constants.CLIMBER_ELEVATOR_UNLATCH_AMOUNT;
                    cl.climberMotor.set(ControlMode.Position, Constants.CLIMBER_ELEVATOR_MAX_SAFE_HEIGHT);
                },
                (cl) -> cl.climberMotor.getSelectedSensorPosition() > cl.data - Constants.CLIMBER_MOTOR_MAX_ERROR &&
                        !cl.elevatorArmContactSwitchA.get() && !cl.elevatorArmContactSwitchB.get(),
                (cl) -> {}
        ),

        /**
         * Extends the solenoid to pivot the robot and cause it to start swinging. Waits a minimum time and until the climber
         * motor has fully extended to it's maximum safe position.
         */
        PIVOT_PIVOT_ARM(
                (cl) -> {
                    cl.setPivotState(PivotState.PIVOTED);
                    cl.data = Timer.getFPGATimestamp();
                },
                (cl) -> Timer.getFPGATimestamp() - cl.data > Constants.ARM_PIVOT_DURATION
                        && cl.climberMotor.getClosedLoopError() < Constants.CLIMBER_MOTOR_MAX_ERROR,
                (cl) -> {}
        ),

        /**
         * Uses the gyro to wait until the robot is swinging towards the field
         */
        WAIT_TILL_EXTENSION_IS_SAFE(
                (cl) -> {},
                (cl) -> {
                    AHRS gyro = RobotTracker.getInstance().getGyro();
                    return gyro.getPitch() < 0 && cl.gyroPitchVelocity < 0; // TODO: tune these values
                },
                (cl) -> {}
        ),

        /**
         * Extends the elevator arm past the safe height. At this point the arm will contact the bar when the robot swings back
         * towards the bars.
         */
        EXTEND_ELEVATOR_ARM_PAST_SAFE_LENGTH(
                (cl) -> {
                    cl.climberMotor.set(ControlMode.Position, Constants.MAX_CLIMBER_EXTENSION);
                },
                (cl) -> cl.climberMotor.getSelectedSensorPosition() > Constants.MAX_CLIMBER_EXTENSION - Constants.CLIMBER_MOTOR_MAX_ERROR,
                (cl) -> {}
        ),

        /**
         * Unpivot the elevator arm so elevator arm is pulled onto the next bar
         */
        UNPIVOT_PIVOT_ARM(
                (cl) -> {
                    cl.setPivotState(PivotState.INLINE);
                    cl.data = Timer.getFPGATimestamp();
                },
                //TODO: Change the time
                (cl) -> Timer.getFPGATimestamp() - cl.data > Constants.ARM_UNPIVOT_DURATION,
                (cl) -> {}
        ),

        /**
         * Waits for the gyro to report that the robot has stopped swinging.
         */
        WAIT_FOR_SWING_STOP(
                (cl) -> {},
                (cl) -> {
                    AHRS gyro = RobotTracker.getInstance().getGyro();
                    return Math.abs(gyro.getPitch() - 20) < 4 && Math.abs(cl.gyroPitchVelocity) < 0.1; //TODO: Tune these values
                },
                (cl) -> {}
        ),

        /**
         * Retracts the elevator arm to until it contacts the bar.
         */
        CONTACT_ELEVATOR_ARM_WITH_NEXT_BAR(
                (cl) -> cl.climberMotor.set(ControlMode.PercentOutput, -Constants.CLIMBER_MOTOR_MAX_OUTPUT),
                (cl) -> cl.elevatorArmContactSwitchA.get() && cl.elevatorArmContactSwitchB.get(),
                (cl) -> cl.stopClimberMotor()
        ),

        /**
         * Unlatches the pivot arm so that the robot is supported by the elevator arm.
         */
        UNLATCH_PIVOT_ARM(
                (cl) -> {
                    cl.setLatchState(LatchState.UNLATCHED);
                    cl.data = Timer.getFPGATimestamp();
                },
                (cl) -> Timer.getFPGATimestamp() - cl.data > Constants.PIVOT_ARM_UNLATCH_DURATION
                        && !cl.pivotingArmContactSwitchA.get() && !cl.pivotingArmContactSwitchB.get(),
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
        super(Constants.CLIMBER_PERIOD);

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
        climberMotor.setNeutralMode(NeutralMode.Brake);
        climberMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.CLIMBER_CURRENT_LIMIT,
                Constants.CLIMBER_CURRENT_LIMIT, 0));

        elevatorArmContactSwitchA = new DigitalInput(Constants.ELEVATOR_ARM_CONTACT_SWITCH_A_DIO_CHANNEL);
        elevatorArmContactSwitchB = new DigitalInput(Constants.ELEVATOR_ARM_CONTACT_SWITCH_B_DIO_CHANNEL);

        pivotingArmContactSwitchA = new DigitalInput(Constants.PIVOTING_ARM_CONTACT_SWITCH_A_DIO_CHANNEL);
        pivotingArmContactSwitchB = new DigitalInput(Constants.PIVOTING_ARM_CONTACT_SWITCH_B_DIO_CHANNEL);
        pivotingArmLatchedSwitchA = new DigitalInput(Constants.PIVOTING_ARM_LATCHED_SWITCH_A_DIO_CHANNEL);
        pivotingArmLatchedSwitchB = new DigitalInput(Constants.PIVOTING_ARM_LATCHED_SWITCH_B_DIO_CHANNEL);

        latchSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.LATCH_SOLENOID_ID); //TODO config solenoid type.
        pivotSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.PIVOT_SOLENOID_ID); //TODO config solenoid type.
        brakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.BRAKE_SOLENOID_ID); //TODO config solenoid type.
    }

    /**
     * Starts the automated climb sequence and deactivates the brake.
     */
    public synchronized void startClimb() {
        climbState = ClimbState.START_CLIMB;
        setBrakeState(BrakeState.FREE);
    }

    /**
     * Stops the climb and resets the climber state. Also activates the brake.
     */
    public synchronized void stopClimb() {
        climbState = ClimbState.IDLE;
        climberMotor.set(ControlMode.PercentOutput, 0);
        setBrakeState(BrakeState.BRAKING);
    }

    /**
     * Pauses the climb.
     * <p>
     * It first stores the current state of the climber in a variable, then sets stops the climber and activates the brake.
     */
    public synchronized void pauseClimb() {
        isPaused = true;
        pausedClimberMode = climberMotor.getControlMode();
        if (pausedClimberMode == ControlMode.PercentOutput) {
            pausedClimberSetpoint = climberMotor.getMotorOutputPercent();
        } else {
            pausedClimberSetpoint = climberMotor.getClosedLoopTarget();
        }
        climberMotor.set(ControlMode.PercentOutput, 0);
        setBrakeState(BrakeState.BRAKING);
    }

    /**
     * Stops the climber motor from moving.
     * <p>
     * It sets the motor to Position Control mode and sets the setpoint to the current position.
     */
    private synchronized void stopClimberMotor() {
        climberMotor.set(ControlMode.Position, climberMotor.getSelectedSensorPosition());
    }

    /**
     * Resumes the climber from a paused state.
     */
    public synchronized void resumeClimb() {
        isPaused = false;
        setBrakeState(BrakeState.FREE);
        climberMotor.set(pausedClimberMode, pausedClimberSetpoint);
    }

    /**
     * Sets the climber in the correct state to initiate a climb and moves the elevator arm to the up above the high bar.
     */
    public synchronized void deployClimb() {
        climberMotor.set(ControlMode.Position, Constants.CLIMBER_DEPLOY_HEIGHT); // TODO: Change position
        setBrakeState(BrakeState.FREE);
        setLatchState(LatchState.UNLATCHED);
        setPivotState(PivotState.INLINE);
    }

    /**
     * Tells the robot to advance the climber to the next state in step by step mode. The robot will still wait till all checks
     * are passing before advancing.
     * <p>
     * Step-by-step mode must be enabled for this method to have an effect.
     */
    public synchronized void advanceStep() {
        advanceStep = true;
    }

    /**
     * Forces the robot to move to the next step immediately. No checks are made to ensure that the robot is in the correct state
     * to move to the next step. <b> The person calling this method needs to ensure that the robot is in the correct state.</b>
     * <p>
     * This method will also work regardless of whether the robot is in step-by-mode or not.
     */
    public synchronized void forceAdvanceStep() {
        advanceStep = true;
        skipChecks = true;
    }

    /**
     * @param stepByStep set to true to enable step-by-step mode.
     */
    public synchronized void setStepByStep(boolean stepByStep) {
        this.stepByStep = stepByStep;
    }

    /**
     * @return true if the climber is in step-by-step mode
     */
    public synchronized boolean isStepByStep() {
        return stepByStep;
    }

    @Override
    public synchronized void update() {
        gyroPitchVelocity = RobotTracker.getInstance().getGyro().getPitch() - lastGyroPitch / Constants.CLIMBER_PERIOD;
        gyroRollVelocity = RobotTracker.getInstance().getGyro().getRoll() - lastGyroRoll / Constants.CLIMBER_PERIOD;

        lastGyroPitch = RobotTracker.getInstance().getGyro().getPitch();
        lastGyroRoll = RobotTracker.getInstance().getGyro().getRoll();

        if (!isPaused) {
            if (climberMotor.getSelectedSensorPosition() < Constants.MIN_CLIMBER_ELEVATOR_HEIGHT
                    && climberMotor.getSelectedSensorVelocity() < 0) {
                stopClimb();
            } else if (climberMotor.getSelectedSensorPosition() > Constants.MAX_CLIMBER_ELEVATOR_HEIGHT
                    && climberMotor.getSelectedSensorVelocity() > 0) {
                stopClimb();
            }

            if (climbState.waitCondition.apply(this)) {
                climbState.endAction.accept(this);
                ranEndAction = true;
            }

            if ((!stepByStep || advanceStep) && (skipChecks || climbState.waitCondition.apply(this))) {
                if (!ranEndAction) climbState.endAction.accept(this);

                climbState = ClimbState.values()[(climbState.ordinal() + 1) % ClimbState.values().length];
                climbState.startAction.accept(this);
                advanceStep = false;
                skipChecks = false;
                ranEndAction = false;
            }
        }
    }

    /**
     * @param percentOutput The percent output to set the climber motor to. Will automatically activate/deactivate the brake
     */
    public void setClimberMotor(double percentOutput) {
        isPaused = true;
        climbState = ClimbState.IDLE;
        setBrakeState(Math.abs(percentOutput) < 1.0E-6 ? BrakeState.BRAKING : BrakeState.FREE);
        climberMotor.set(ControlMode.PercentOutput, percentOutput);
    }
    
    /**
     * Toggles the latch that is on the pivot arm.
     */
    public void toggleLatch() {
        isPaused = true;
        climbState = ClimbState.IDLE;
        latchSolenoid.set(!latchSolenoid.get());
    }

    /**
     * Toggles the pivot arm in and out. Also pauses the climber motor.
     */
    public void togglePivot() {
        isPaused = true;
        climbState = ClimbState.IDLE;
        pivotSolenoid.set(!pivotSolenoid.get());
    }

    @Override
    public void selfTest() {

    }

    public ClimbState getClimbState() {
        return climbState;
    }

    @Override
    public void logData() {
        logData("Climber Motor Position", climberMotor.getSelectedSensorPosition());
        logData("Climber Motor Velocity", climberMotor.getSelectedSensorVelocity());
        logData("Climber Motor Percent Output", climberMotor.getMotorOutputPercent());
        logData("Climber Motor Current", climberMotor.getStatorCurrent());
        logData("Climber Motor 2 Current", climberMotor2.getStatorCurrent());
        logData("Climber Motor Current Limit", Constants.CLIMBER_CURRENT_LIMIT);

        logData("Elevator Arm Contact Switch A", elevatorArmContactSwitchA.get());
        logData("Elevator Arm Contact Switch B", elevatorArmContactSwitchB.get());
        logData("Pivot Arm Contact Switch A", pivotingArmContactSwitchA.get());
        logData("Pivot Arm Contact Switch B", pivotingArmContactSwitchB.get());
        logData("Pivoting Arm Contact Switch A", pivotingArmLatchedSwitchA.get());
        logData("Pivoting Arm Contact Switch B", pivotingArmLatchedSwitchB.get());

        logData("Pivot Solenoid State", getPivotState().toString());
        logData("Latch Solenoid State", getLatchState().toString());
        logData("Brake Solenoid State", getBrakeState().toString());

        logData("Gyro Pitch", RobotTracker.getInstance().getGyro().getPitch());
        logData("Gyro Pitch Velocity", gyroPitchVelocity);
        logData("Gyro Roll", RobotTracker.getInstance().getGyro().getRoll());
        logData("Gyro Roll Velocity", gyroRollVelocity);

        logData("Climber Is Paused", isPaused);
        logData("Climber Is Step By Step", stepByStep);
        logData("Current State", climbState.toString());
    }

    @Override
    public void close() throws Exception {

    }
}
