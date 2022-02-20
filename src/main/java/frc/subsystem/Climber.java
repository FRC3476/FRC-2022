package frc.subsystem;

import com.ctre.phoenix.motorcontrol.*;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.utility.Timer;
import frc.utility.controllers.LazyTalonSRX;
import org.jetbrains.annotations.NotNull;

import java.util.function.Consumer;
import java.util.function.Function;

import static frc.robot.Constants.CLIMBER_ENCODER_TICKS_PER_INCH;
import static frc.utility.Pneumatics.getPneumaticsHub;

class ClimbState {
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
}

public final class Climber extends AbstractSubsystem {
    private static final Climber INSTANCE = new Climber();

    public static Climber getInstance() {
        return INSTANCE;
    }

    private final @NotNull LazyTalonSRX climberMotor;
    private final @NotNull LazyTalonSRX climberMotor2;

    private final @NotNull DigitalInput elevatorArmContactSwitchA;
    private final @NotNull DigitalInput elevatorArmContactSwitchB;

    private final @NotNull DigitalInput pivotingArmContactSwitchA;
    private final @NotNull DigitalInput pivotingArmContactSwitchB;
    private final @NotNull DigitalInput pivotingArmLatchedSwitchA;
    private final @NotNull DigitalInput pivotingArmLatchedSwitchB;

    private final @NotNull Solenoid latchSolenoid;
    private final @NotNull Solenoid pivotSolenoid;
    private final @NotNull Solenoid brakeSolenoid;

    private double data;

    private @NotNull ClimbStatePair climbStatePair = ClimbStatePair.IDLE;
    private boolean isPaused = false;
    private double pausedClimberSetpoint;
    private ControlMode pausedClimberMode;

    private boolean stepByStep = true;
    private boolean advanceStep = false;
    private boolean skipChecks = false;
    private boolean ranEndAction = false;

    private double gyroPitchVelocity = 0;
    private double lastGyroPitch;
    private double gyroRollVelocity = 0;
    private double lastGyroRoll;

    public enum ClawState {
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

    public void setClawState(ClawState clawState) {
        latchSolenoid.set(clawState == ClawState.UNLATCHED);
    }

    public void setPivotState(PivotState pivotState) {
        pivotSolenoid.set(pivotState == PivotState.PIVOTED);
    }

    public BrakeState getBrakeState() {
        return brakeSolenoid.get() ? BrakeState.BRAKING : BrakeState.FREE;
    }

    public ClawState getClawState() {
        return latchSolenoid.get() ? ClawState.UNLATCHED : ClawState.LATCHED;
    }

    public PivotState getPivotState() {
        return pivotSolenoid.get() ? PivotState.PIVOTED : PivotState.INLINE;
    }

    public enum ClimbStatePair {
        IDLE(new ClimbState((cl) -> {}, (cl) -> false, (cl) -> {}),
                new ClimbState((cl) -> {}, (cl) -> false, (cl) -> {})
        ),

        /**
         * Will immediately transition to the next state to start the climb sequence.
         */
        START_CLIMB(new ClimbState((cl) -> {}, (cl) -> true, (cl) -> {}),
                new ClimbState((cl) -> {}, (cl) -> false, (cl) -> {})
        ),

        /**
         * Lowers the elevator arm until the contact switch is pressed on the bar.
         */
        LOWER_ELEVATOR_ARM_TILL_PIVOT_ARM_CONTACT(
                new ClimbState((cl) -> {
                    cl.climberMotor.set(ControlMode.PercentOutput, -Constants.CLIMBER_MOTOR_MAX_OUTPUT);
                    cl.setClawState(ClawState.UNLATCHED);
                },
                        (cl) -> cl.isPivotArmContactingBar(),
                        (cl) -> cl.stopClimberMotor()
                ),

                new ClimbState((cl) -> {
                    cl.climberMotor.set(ControlMode.Position, Constants.CLIMBER_GRAB_ON_FIRST_BAR_EXTENSION);
                    cl.setClawState(ClawState.UNLATCHED);
                },
                        (cl) -> false,
                        (cl) -> cl.stopClimberMotor())
        ),

        /**
         * Extends the solenoid to latch the pivoting arm onto the bar. Waits until the latch switch is pressed.
         */
        LATCH_PIVOT_ARM(
                new ClimbState((cl) -> cl.setClawState(ClawState.LATCHED),
                        (cl) -> cl.pivotingArmLatchedSwitchA.get() && cl.pivotingArmLatchedSwitchB.get(),
                        (cl) -> {}),

                new ClimbState((cl) -> cl.setClawState(ClawState.LATCHED),
                        (cl) -> false,
                        (cl) -> {})
        ),

        /**
         * Moves the elevator arm up to the maximum safe height. Will continue to the next state once the elevator is no longer
         * supporting the robot.
         */
        MOVE_WEIGHT_TO_PIVOT_ARM(
                new ClimbState((cl) -> {
                    //position when it is considered "unlatched"
                    cl.data = cl.climberMotor.getSelectedSensorPosition() + Constants.CLIMBER_ELEVATOR_UNLATCH_AMOUNT;
                    cl.climberMotor.set(ControlMode.Position, Constants.CLIMBER_ELEVATOR_MAX_SAFE_HEIGHT);
                },
                        (cl) -> cl.climberMotor.getSelectedSensorPosition() > cl.data - Constants.CLIMBER_MOTOR_MAX_ERROR &&
                                !cl.elevatorArmContactSwitchA.get() && !cl.elevatorArmContactSwitchB.get(),
                        (cl) -> {}),

                new ClimbState((cl) -> {
                    //position when it is considered "unlatched"
                    cl.data = cl.climberMotor.getSelectedSensorPosition() + Constants.CLIMBER_ELEVATOR_UNLATCH_AMOUNT;
                    cl.climberMotor.set(ControlMode.Position, Constants.CLIMBER_ELEVATOR_MAX_SAFE_HEIGHT);
                },
                        (cl) -> false,
                        (cl) -> {})
        ),

        /**
         * Extends the solenoid to pivot the robot and cause it to start swinging. Waits a minimum time and until the climber
         * motor has fully extended to it's maximum safe position.
         */
        PIVOT_PIVOT_ARM(
                new ClimbState((cl) -> {
                    cl.setPivotState(PivotState.PIVOTED);
                    cl.data = Timer.getFPGATimestamp();
                },
                        (cl) -> Timer.getFPGATimestamp() - cl.data > Constants.ARM_PIVOT_DURATION
                                && cl.climberMotor.getClosedLoopError() < Constants.CLIMBER_MOTOR_MAX_ERROR,
                        (cl) -> {}),

                new ClimbState((cl) -> {
                    cl.setPivotState(PivotState.PIVOTED);
                    cl.data = Timer.getFPGATimestamp();
                },
                        (cl) -> false,
                        (cl) -> {})
        ),

        /**
         * Uses the gyro to wait until the robot is swinging towards the field
         */
        WAIT_TILL_EXTENSION_IS_SAFE(
                new ClimbState((cl) -> {},
                        (cl) -> {
                            AHRS gyro = RobotTracker.getInstance().getGyro();
                            return gyro.getPitch() < 0 && cl.gyroPitchVelocity < 0; // TODO: tune these values
                        },
                        (cl) -> {}),

                new ClimbState((cl) -> {},
                        (cl) -> false
                        ,
                        (cl) -> {})
        ),

        /**
         * Extends the elevator arm past the safe height. At this point the arm will contact the bar when the robot swings back
         * towards the bars.
         */
        EXTEND_ELEVATOR_ARM_PAST_SAFE_LENGTH(
                new ClimbState((cl) -> {
                    cl.climberMotor.set(ControlMode.Position, Constants.MAX_CLIMBER_EXTENSION);
                },
                        (cl) -> cl.climberMotor.getSelectedSensorPosition() > Constants.MAX_CLIMBER_EXTENSION - Constants.CLIMBER_MOTOR_MAX_ERROR,
                        (cl) -> {}),

                new ClimbState((cl) -> {
                    cl.climberMotor.set(ControlMode.Position, Constants.MAX_CLIMBER_EXTENSION);
                },
                        (cl) -> false,
                        (cl) -> {})
        ),

        /**
         * Unpivot the elevator arm so elevator arm is pulled onto the next bar
         */
        UNPIVOT_PIVOT_ARM(
                new ClimbState((cl) -> {
                    cl.setPivotState(PivotState.INLINE);
                    cl.data = Timer.getFPGATimestamp();
                },
                        //TODO: Change the time
                        (cl) -> Timer.getFPGATimestamp() - cl.data > Constants.ARM_UNPIVOT_DURATION,
                        (cl) -> {}),

                new ClimbState((cl) -> {
                    cl.setPivotState(PivotState.INLINE);
                    cl.data = Timer.getFPGATimestamp();
                },
                        //TODO: Change the time
                        (cl) -> false,
                        (cl) -> {})
        ),

        /**
         * Waits for the gyro to report that the robot has stopped swinging.
         */
        WAIT_FOR_SWING_STOP(
                new ClimbState((cl) -> {},
                        (cl) -> {
                            AHRS gyro = RobotTracker.getInstance().getGyro();
                            return Math.abs(gyro.getPitch() - 20) < 4 && Math.abs(
                                    cl.gyroPitchVelocity) < 0.1; //TODO: Tune these values
                        },
                        (cl) -> {}),

                new ClimbState((cl) -> {},
                        (cl) -> false,
                        (cl) -> {})
        ),

        /**
         * Retracts the elevator arm to until it contacts the bar.
         */
        CONTACT_ELEVATOR_ARM_WITH_NEXT_BAR(
                new ClimbState((cl) -> cl.climberMotor.set(ControlMode.PercentOutput, -Constants.CLIMBER_MOTOR_MAX_OUTPUT),
                        (cl) -> cl.isElevatorArmContactingBar(),
                        (cl) -> cl.stopClimberMotor()),

                new ClimbState((cl) -> cl.climberMotor.set(ControlMode.Position, Constants.CLIMBER_GRAB_ON_NEXT_BAR_EXTENSION),
                        (cl) -> false,
                        (cl) -> cl.stopClimberMotor())
        ),

        /**
         * Unlatches the pivot arm so that the robot is supported by the elevator arm.
         */
        UNLATCH_PIVOT_ARM(new ClimbState(
                (cl) -> {
                    cl.setClawState(ClawState.UNLATCHED);
                    cl.data = Timer.getFPGATimestamp();
                },
                (cl) -> {
                    if (cl.pivotingArmLatchedSwitchA.get() && cl.pivotingArmLatchedSwitchB.get()) {
                        cl.data = Timer.getFPGATimestamp();
                    }
                    return Timer.getFPGATimestamp() - cl.data > Constants.PIVOT_ARM_UNLATCH_DURATION
                            && !cl.pivotingArmContactSwitchA.get() && !cl.pivotingArmContactSwitchB.get();
                },
                (cl) -> {}),
                new ClimbState(
                        (cl) -> {
                            cl.setClawState(ClawState.UNLATCHED);
                            cl.data = Timer.getFPGATimestamp();
                        },
                        (cl) -> false,
                        (cl) -> {})
        );

        ClimbStatePair(ClimbState automatic, ClimbState stepbystep) {
            this.automatic = automatic;
            this.stepbystep = stepbystep;
        }

        ClimbState automatic;
        ClimbState stepbystep;
    }

    double otherPivotingArmMustContactByTime = Double.MAX_VALUE;

    /**
     * Will pause the climb if it detects that only one pivot arm is in contact with the bar for some time.
     *
     * @return true if both pivot arms are in contact with the bar
     */
    private boolean isPivotArmContactingBar() {
        if (pivotingArmContactSwitchA.get() || pivotingArmContactSwitchB.get()) {
            if (otherPivotingArmMustContactByTime > Timer.getFPGATimestamp() + Constants.MAX_ALLOW_ONLY_ONE_SWITCH_CONTACT_TIME) {
                otherPivotingArmMustContactByTime = Timer.getFPGATimestamp() + Constants.MAX_ALLOW_ONLY_ONE_SWITCH_CONTACT_TIME;
            } else if (Timer.getFPGATimestamp() > otherPivotingArmMustContactByTime) {
                pauseClimb();
            }
        } else if (pivotingArmContactSwitchA.get() && pivotingArmContactSwitchB.get()) {
            return true;
        } else {
            otherPivotingArmMustContactByTime = Double.MAX_VALUE;
        }
        return false;
    }

    double otherElevatorArmMustContactByTime = Double.MAX_VALUE;

    /**
     * Will pause the climb if it detects that only one pivot arm is in contact with the bar for some time.
     *
     * @return true if both pivot arms are in contact with the bar
     */
    private boolean isElevatorArmContactingBar() {
        if (elevatorArmContactSwitchA.get() || elevatorArmContactSwitchB.get()) {
            if (otherElevatorArmMustContactByTime > Timer.getFPGATimestamp() + Constants.MAX_ALLOW_ONLY_ONE_SWITCH_CONTACT_TIME) {
                otherElevatorArmMustContactByTime = Timer.getFPGATimestamp() + Constants.MAX_ALLOW_ONLY_ONE_SWITCH_CONTACT_TIME;
            } else if (Timer.getFPGATimestamp() > otherElevatorArmMustContactByTime) {
                pauseClimb();
            }
        } else if (elevatorArmContactSwitchA.get() && elevatorArmContactSwitchB.get()) {
            return true;
        } else {
            otherElevatorArmMustContactByTime = Double.MAX_VALUE;
        }
        return false;
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
        climberMotor.configPeakOutputReverse(-Constants.CLIMBER_MOTOR_MAX_OUTPUT);
        climberMotor.setNeutralMode(NeutralMode.Brake);
        climberMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.CLIMBER_CURRENT_LIMIT,
                Constants.CLIMBER_CURRENT_LIMIT, 0));

        climberMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20); // Default is 10ms
        climberMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 25); // Default is 10ms
        climberMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 100); // Default is 50ms
        climberMotor.setControlFramePeriod(ControlFrame.Control_3_General, 25);
        climberMotor.setControlFramePeriod(ControlFrame.Control_4_Advanced, 25);
        climberMotor.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 500);

        climberMotor2.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100); // Default is 10ms
        climberMotor2.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 100); // Default is 10ms
        climberMotor2.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 100); // Default is 50ms
        climberMotor2.setControlFramePeriod(ControlFrame.Control_3_General, 25);
        climberMotor2.setControlFramePeriod(ControlFrame.Control_4_Advanced, 25);
        climberMotor2.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 500);

        elevatorArmContactSwitchA = new DigitalInput(Constants.ELEVATOR_ARM_CONTACT_SWITCH_A_DIO_CHANNEL);
        elevatorArmContactSwitchB = new DigitalInput(Constants.ELEVATOR_ARM_CONTACT_SWITCH_B_DIO_CHANNEL);

        pivotingArmContactSwitchA = new DigitalInput(Constants.PIVOTING_ARM_CONTACT_SWITCH_A_DIO_CHANNEL);
        pivotingArmContactSwitchB = new DigitalInput(Constants.PIVOTING_ARM_CONTACT_SWITCH_B_DIO_CHANNEL);
        pivotingArmLatchedSwitchA = new DigitalInput(Constants.PIVOTING_ARM_LATCHED_SWITCH_A_DIO_CHANNEL);
        pivotingArmLatchedSwitchB = new DigitalInput(Constants.PIVOTING_ARM_LATCHED_SWITCH_B_DIO_CHANNEL);

        latchSolenoid = getPneumaticsHub().makeSolenoid(Constants.LATCH_SOLENOID_ID);
        pivotSolenoid = getPneumaticsHub().makeSolenoid(Constants.PIVOT_SOLENOID_ID);
        brakeSolenoid = getPneumaticsHub().makeSolenoid(Constants.BRAKE_SOLENOID_ID);

        climberMotor.setInverted(true);
        climberMotor2.setInverted(true);
    }

    /**
     * Starts the automated climb sequence and deactivates the brake.
     */
    public synchronized void startClimb() {
        climbStatePair = ClimbStatePair.START_CLIMB;
        setBrakeState(BrakeState.FREE);
    }

    /**
     * Stops the climb and resets the climber state. Also activates the brake.
     */
    public synchronized void stopClimb() {
        climbStatePair = ClimbStatePair.IDLE;
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
        otherPivotingArmMustContactByTime = Double.MAX_VALUE;
    }

    /**
     * Sets the climber in the correct state to initiate a climb and moves the elevator arm to the up above the high bar.
     */
    public synchronized void deployClimb() {
        climberMotor.set(ControlMode.Position, Constants.CLIMBER_DEPLOY_HEIGHT);
        setBrakeState(BrakeState.FREE);
        setClawState(ClawState.UNLATCHED);
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
        ClimbState currentClimbState;

        if (stepByStep) {
            currentClimbState = climbStatePair.stepbystep;
        } else {
            currentClimbState = climbStatePair.automatic;
        }

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

            if (currentClimbState.waitCondition.apply(this)) {
                currentClimbState.endAction.accept(this);
                ranEndAction = true;
            }

            if (skipChecks || currentClimbState.waitCondition.apply(this)) {
                //if (!ranEndAction) currentClimbState.endAction.accept(this);

                climbStatePair = ClimbStatePair.values()[(climbStatePair.ordinal() + 1) % ClimbStatePair.values().length];

                if (stepByStep) {
                    currentClimbState = climbStatePair.stepbystep;
                } else {
                    currentClimbState = climbStatePair.automatic;
                }

                currentClimbState.startAction.accept(this);
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
        climbStatePair = ClimbStatePair.IDLE;
        setBrakeState(Math.abs(percentOutput) < 1.0E-2 ? BrakeState.BRAKING : BrakeState.FREE);
        climberMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    /**
     * Toggles the latch that is on the pivot arm.
     */
    public void toggleClaw() {
        isPaused = true;
        climbStatePair = ClimbStatePair.IDLE;
        setClawState(getClawState() == ClawState.UNLATCHED ? ClawState.LATCHED : ClawState.UNLATCHED);
    }

    /**
     * Toggles the pivot arm in and out. Also pauses the climber motor.
     */
    public void togglePivot() {
        isPaused = true;
        climbStatePair = ClimbStatePair.IDLE;
        setPivotState(getPivotState() == PivotState.INLINE ? PivotState.PIVOTED : PivotState.INLINE);
    }

    @Override
    public void selfTest() {

    }

    public @NotNull ClimbStatePair getClimbStatePair() {
        return climbStatePair;
    }

    @Override
    public void logData() {
        logData("Climber Motor Position", climberMotor.getSelectedSensorPosition());
        logData("Climber Motor Position IN", climberMotor.getSelectedSensorPosition() / CLIMBER_ENCODER_TICKS_PER_INCH);
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

        logData("Pivot Solenoid State", getPivotState().toString(), true);
        logData("Latch Solenoid State", getClawState().toString(), true);
        logData("Brake Solenoid State", getBrakeState().toString(), true);

        logData("Gyro Pitch", RobotTracker.getInstance().getGyro().getPitch(), true);
        logData("Gyro Pitch Velocity", gyroPitchVelocity, true);
        logData("Gyro Roll", RobotTracker.getInstance().getGyro().getRoll(), true);
        logData("Gyro Roll Velocity", gyroRollVelocity, true);

        logData("Climber Is Paused", isPaused, true);
        logData("Climber Is Step By Step", stepByStep, true);
        logData("Current State", climbStatePair.toString(), true);
    }

    @Override
    public void close() throws Exception {

    }
}
