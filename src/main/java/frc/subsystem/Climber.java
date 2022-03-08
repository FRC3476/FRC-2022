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

import static frc.robot.Constants.*;
import static frc.utility.Pneumatics.getPneumaticsHub;

class ClimbStep {
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
     * true if this step causes something on the robot to move
     */
    final boolean runInStepByStep;

    /**
     * @param startAction     The action to perform when the state is entered
     * @param waitCondition   The condition to wait for before transitioning to the next state (true = continue, false = wait)
     * @param endAction       The action to perform when the state is exited
     * @param runInStepByStep if this step causes something on the robot to move
     */
    ClimbStep(Consumer<Climber> startAction, Function<Climber, Boolean> waitCondition, Consumer<Climber> endAction,
              boolean runInStepByStep) {
        this.startAction = startAction;
        this.waitCondition = waitCondition;
        this.endAction = endAction;
        this.runInStepByStep = runInStepByStep;
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

    private @NotNull Climber.ClimbState climbState = ClimbState.IDLE;

    private boolean isPaused = false;
    private double pausedClimberSetpoint;
    private ControlMode pausedClimberMode;

    private boolean sensorClimb = true;
    private boolean stepByStep = true;
    private boolean advanceStep = false;
    private boolean skipChecks = false;
    private boolean ranEndAction = false;

    private double gyroPitchVelocity = 0;
    private double lastGyroPitch;
    private double gyroRollVelocity = 0;
    private double lastGyroRollVelocity;
    private double lastGyroRoll;
    private double gyroRollAccel = 0;

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

    public enum ClimbState {
        IDLE(
                new ClimbStep(
                        (cl) -> {},
                        (cl) -> false,
                        (cl) -> {},
                        true
                ),

                new ClimbStep(
                        (cl) -> {},
                        (cl) -> false,
                        (cl) -> {},
                        true
                )
        ),

        /**
         * Will immediately transition to the next state to start the climb sequence.
         */
        START_CLIMB(
                new ClimbStep(
                        (cl) -> {},
                        (cl) -> true,
                        (cl) -> {},
                        false
                ),

                new ClimbStep(
                        (cl) -> {},
                        (cl) -> {
                            AHRS gyro = RobotTracker.getInstance().getGyro();
                            if (Math.abs(gyro.getRoll()) < 5 && Math.abs(cl.gyroRollVelocity) < 2) {
                                return true;
                            } else {
                                return gyro.getRoll() > 10 && cl.gyroRollVelocity < 0.1; //TODO: Tune these values
                            }
                        },
                        (cl) -> {},
                        false
                )
        ),

        /**
         * Lowers the elevator arm until the contact switch is pressed on the bar.
         */
        LOWER_ELEVATOR_ARM_TILL_PIVOT_ARM_CONTACT(
                new ClimbStep(
                        (cl) -> {
                            cl.climberMotor.set(ControlMode.PercentOutput, -Constants.CLIMBER_MOTOR_MAX_OUTPUT);
                            cl.setClawState(ClawState.UNLATCHED);
                        },
                        Climber::isPivotArmContactingBar,
                        Climber::stopClimberMotor,
                        true
                ),

                new ClimbStep(
                        (cl) -> {
                            cl.climberMotor.set(ControlMode.Position, Constants.CLIMBER_GRAB_ON_FIRST_BAR_EXTENSION);
                            cl.setClawState(ClawState.UNLATCHED);
                        },
                        (cl) -> Math.abs(
                                Constants.CLIMBER_GRAB_ON_FIRST_BAR_EXTENSION - cl.climberMotor.getSelectedSensorPosition())
                                < Constants.CLIMBER_MOTOR_MAX_ERROR,
                        (cl) -> {},
                        true
                )
        ),

        /**
         * Extends the solenoid to latch the pivoting arm onto the bar. Waits until the latch switch is pressed.
         */
        LATCH_PIVOT_ARM(
                new ClimbStep(
                        (cl) -> cl.setClawState(ClawState.LATCHED),
                        Climber::isPivotingArmLatched,
                        (cl) -> {},
                        true
                ),

                new ClimbStep(
                        (cl) -> {
                            cl.setClawState(ClawState.LATCHED);
                            cl.data = Timer.getFPGATimestamp() + 1;
                        },
                        (cl) -> Timer.getFPGATimestamp() > cl.data,
                        (cl) -> {},
                        true
                )
        ),

        /**
         * Moves the elevator arm up to the maximum safe height. Will continue to the next state once the elevator is no longer
         * supporting the robot.
         */
        MOVE_WEIGHT_TO_PIVOT_ARM(
                new ClimbStep(
                        (cl) -> {
                            //position when it is considered "unlatched"
                            cl.data = cl.climberMotor.getSelectedSensorPosition() + Constants.CLIMBER_ELEVATOR_UNLATCH_AMOUNT;
                            cl.climberMotor.set(ControlMode.Position, Constants.CLIMBER_ELEVATOR_MAX_SAFE_HEIGHT);
                        },
                        (cl) -> cl.climberMotor.getSelectedSensorPosition() > cl.data - Constants.CLIMBER_MOTOR_MAX_ERROR &&
                                !cl.elevatorArmContactSwitchA.get() && !cl.elevatorArmContactSwitchB.get(),
                        (cl) -> {},
                        true
                ),

                new ClimbStep(
                        (cl) -> {
                            cl.data = cl.climberMotor.getSelectedSensorPosition() + Constants.CLIMBER_ELEVATOR_UNLATCH_AMOUNT;
                            cl.climberMotor.set(ControlMode.Position, Constants.CLIMBER_ELEVATOR_MAX_SAFE_HEIGHT);
                        },
                        (cl) -> cl.climberMotor.getSelectedSensorPosition() > cl.data - Constants.CLIMBER_MOTOR_MAX_ERROR,
                        (cl) -> {},
                        true
                )
        ),

        /**
         * Extends the solenoid to pivot the robot and cause it to start swinging. Waits a minimum time and until the climber
         * motor has fully extended to it's maximum safe position.
         */
        PIVOT_PIVOT_ARM(
                new ClimbStep(
                        (cl) -> {
                            cl.setPivotState(PivotState.PIVOTED);
                            cl.data = Timer.getFPGATimestamp();
                        },
                        (cl) -> Timer.getFPGATimestamp() - cl.data > Constants.ARM_PIVOT_DURATION
                                && cl.climberMotor.getClosedLoopError() < Constants.CLIMBER_MOTOR_MAX_ERROR,
                        (cl) -> {},
                        true
                ),

                new ClimbStep(
                        (cl) -> {
                            cl.setPivotState(PivotState.PIVOTED);
                            cl.data = Timer.getFPGATimestamp();
                        },
                        (cl) -> Timer.getFPGATimestamp() - cl.data > Constants.ARM_PIVOT_DURATION
                                && cl.climberMotor.getClosedLoopError() < Constants.CLIMBER_MOTOR_MAX_ERROR,
                        (cl) -> {},
                        true
                )
        ),

        /**
         * Uses the gyro to wait until the robot is swinging towards the field
         */
        WAIT_TILL_EXTENSION_IS_SAFE(
                new ClimbStep(
                        (cl) -> {},
                        (cl) -> {
                            AHRS gyro = RobotTracker.getInstance().getGyro();
                            return cl.gyroPitchVelocity > 0 && gyro.getRoll() > 42.5;
                        }, // TODO: tune these values
                        (cl) -> {},
                        false
                ),

                new ClimbStep(
                        (cl) -> {},
                        (cl) -> {
                            AHRS gyro = RobotTracker.getInstance().getGyro();
                            return cl.gyroPitchVelocity > 0 && gyro.getRoll() > 42.5;
                        },
                        (cl) -> {},
                        false
                )
        ),

        /**
         * Extends the elevator arm past the safe height. At this point the arm will contact the bar when the robot swings back
         * towards the bars.
         */
        EXTEND_ELEVATOR_ARM_PAST_SAFE_LENGTH(
                new ClimbStep(
                        (cl) -> cl.climberMotor.set(ControlMode.Position, Constants.MAX_CLIMBER_EXTENSION),
                        (cl) -> cl.climberMotor.getSelectedSensorPosition() > Constants.MAX_CLIMBER_EXTENSION - Constants.CLIMBER_MOTOR_MAX_ERROR,
                        (cl) -> {},
                        true
                ),

                new ClimbStep(
                        (cl) -> cl.climberMotor.set(ControlMode.Position, Constants.MAX_CLIMBER_EXTENSION),
                        (cl) -> cl.climberMotor.getSelectedSensorPosition() > Constants.MAX_CLIMBER_EXTENSION - Constants.CLIMBER_MOTOR_MAX_ERROR,
                        (cl) -> {},
                        true
                )
        ),

        /**
         * Unpivot the elevator arm so elevator arm is pulled onto the next bar
         */
        UNPIVOT_PIVOT_ARM(
                new ClimbStep(
                        (cl) -> {
                            cl.setPivotState(PivotState.INLINE);
                            cl.data = Timer.getFPGATimestamp();
                        },
                        //TODO: Change the time
                        (cl) -> Timer.getFPGATimestamp() - cl.data > Constants.ARM_UNPIVOT_DURATION,
                        (cl) -> {},
                        true
                ),

                new ClimbStep(
                        (cl) -> {
                            cl.setPivotState(PivotState.INLINE);
                            cl.data = Timer.getFPGATimestamp();
                        },
                        //TODO: Change the time
                        (cl) -> Timer.getFPGATimestamp() - cl.data > Constants.ARM_UNPIVOT_DURATION,
                        (cl) -> {},
                        true
                )
        ),

        /**
         * Waits for the gyro to report that the robot has stopped swinging.
         */
        WAIT_FOR_SWING_STOP(
                new ClimbStep(
                        (cl) -> {},
                        (cl) -> {
                            AHRS gyro = RobotTracker.getInstance().getGyro();
                            return gyro.getRoll() < 40.5 && Math.abs(cl.gyroRollVelocity) < 2.5; //TODO: Tune these values
                        },
                        (cl) -> {},
                        false
                ),

                new ClimbStep(
                        (cl) -> {
                            cl.data = Double.MAX_VALUE;
                        },
                        (cl) -> {
                            AHRS gyro = RobotTracker.getInstance().getGyro();
                            if (gyro.getRoll() < 41 && Math.abs(cl.gyroRollVelocity) < 2.5) {
                                if (cl.data > Timer.getFPGATimestamp() + 0.3) cl.data = Timer.getFPGATimestamp() + 0.3;
                                return cl.data < Timer.getFPGATimestamp();
                            } else {
                                cl.data = Double.MAX_VALUE;
                            }
                            return false;
                        },
                        (cl) -> {},
                        false
                )
        ),

        /**
         * Retracts the elevator arm to until it contacts the bar.
         */
        CONTACT_ELEVATOR_ARM_WITH_NEXT_BAR(
                new ClimbStep(
                        (cl) -> cl.climberMotor.set(ControlMode.PercentOutput, -Constants.CLIMBER_MOTOR_MAX_OUTPUT),
                        Climber::isElevatorArmContactingBar,
                        Climber::stopClimberMotor,
                        true
                ),

                new ClimbStep(
                        (cl) -> {
                            cl.climberMotor.set(ControlMode.Position, Constants.CLIMBER_GRAB_ON_NEXT_BAR_EXTENSION);
                            Drive.getInstance().setSwerveModuleStates(Constants.SWERVE_MODULE_STATE_FORWARD, true);
                        },
                        (cl) -> cl.climberMotor.getSelectedSensorPosition() - (0.3 * CLIMBER_ENCODER_TICKS_PER_INCH) < CLIMBER_GRAB_ON_NEXT_BAR_EXTENSION,
                        // TODO: Determine if we want this
                        (cl) -> {},
                        true
                )
        ),

        /**
         * Unlatches the pivot arm so that the robot is supported by the elevator arm.
         */
        UNLATCH_PIVOT_ARM(
                new ClimbStep(
                        (cl) -> {
                            cl.setClawState(ClawState.UNLATCHED);
                            cl.data = Timer.getFPGATimestamp();
                        },
                        (cl) -> {
                            if (cl.isPivotingArmLatchedSwitchA() || cl.isElevatorArmContactSwitchB()) {
                                cl.data = Timer.getFPGATimestamp();
                            }
                            return Timer.getFPGATimestamp() - cl.data > Constants.PIVOT_ARM_UNLATCH_DURATION
                                    && !cl.pivotingArmContactSwitchA.get() && !cl.pivotingArmContactSwitchB.get();
                        },
                        (cl) -> {},
                        true
                ),

                new ClimbStep(
                        (cl) -> {
                            cl.setClawState(ClawState.UNLATCHED);
                            cl.data = Timer.getFPGATimestamp();
                        },
                        (cl) -> Timer.getFPGATimestamp() - cl.data > Constants.PIVOT_ARM_UNLATCH_DURATION,
                        (cl) -> {},
                        true
                )
        );

        /**
         * @param sensorClimb   The climb step to use when doing an automatic climb
         * @param positionClimb The climb step to use when doing a manual climb, step by step
         */
        ClimbState(ClimbStep sensorClimb, ClimbStep positionClimb) {
            this.sensorClimb = sensorClimb;
            this.positionClimb = positionClimb;
        }

        /**
         * The climb step to use when doing an automatic climb
         */
        final ClimbStep sensorClimb;

        /**
         * The climb step to use when doing a manual climb, step by step
         */
        final ClimbStep positionClimb;
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

    private double timesRun;


    private Climber() {
        super(Constants.CLIMBER_PERIOD, 1);

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
        climberMotor2.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.CLIMBER_CURRENT_LIMIT,
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

        climberMotor.setSelectedSensorPosition(0);

        timesRun = 0;
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
        timesRun = 0;
        hasStalledIntoBottom = false;
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
     * @param sensorClimb set to true to enable step-by-step mode.
     */
    public synchronized void setSensorClimb(boolean sensorClimb) {
        this.sensorClimb = sensorClimb;
    }

    public synchronized void setStepByStep(boolean stepByStep) {
        this.stepByStep = stepByStep;
    }

    public synchronized boolean isStepByStep() {
        return stepByStep;
    }

    /**
     * @return true if the climber is in step-by-step mode
     */
    public synchronized boolean isSensorClimb() {
        return sensorClimb;
    }

    @Override
    public synchronized void update() {
        ClimbStep currentClimbStep;

        if (sensorClimb) {
            currentClimbStep = climbState.positionClimb;
        } else {
            currentClimbStep = climbState.sensorClimb;
        }

        double lastGyroRollVelocity = gyroRollVelocity;

        gyroPitchVelocity = (RobotTracker.getInstance().getGyro().getPitch() - lastGyroPitch) / ((double) CLIMBER_PERIOD / 1000);
        gyroRollVelocity = (RobotTracker.getInstance().getGyro().getRoll() - lastGyroRoll) / ((double) CLIMBER_PERIOD / 1000);

        lastGyroPitch = RobotTracker.getInstance().getGyro().getPitch();
        lastGyroRoll = RobotTracker.getInstance().getGyro().getRoll();

        gyroRollAccel = (gyroRollVelocity - lastGyroRollVelocity) / ((double) CLIMBER_PERIOD / 1000);


        if (!isPaused) {
            if (climberMotor.getSelectedSensorPosition() < Constants.MIN_CLIMBER_ELEVATOR_HEIGHT
                    && climberMotor.getSelectedSensorVelocity() < 0) {
                stopClimb();
            } else if (climberMotor.getSelectedSensorPosition() > Constants.MAX_CLIMBER_ELEVATOR_HEIGHT
                    && climberMotor.getSelectedSensorVelocity() > 0) {
                stopClimb();
            }

            if (currentClimbStep.waitCondition.apply(this)) {
                currentClimbStep.endAction.accept(this);
                ranEndAction = true;
            }

            if (skipChecks || (currentClimbStep.waitCondition.apply(this) && !stepByStep)) {
                if (!ranEndAction) currentClimbStep.endAction.accept(this);
                do {
                    climbState = ClimbState.values()[(climbState.ordinal() + 1) % ClimbState.values().length];
                    if (climbState == ClimbState.IDLE && timesRun < 1) {
                        climbState = ClimbState.START_CLIMB;
                        timesRun++;
                    }

                    if (sensorClimb) {
                        currentClimbStep = climbState.positionClimb;
                    } else {
                        currentClimbStep = climbState.sensorClimb;
                    }
                } while (!currentClimbStep.runInStepByStep && stepByStep);

                currentClimbStep.startAction.accept(this);
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
        setBrakeState(Math.abs(percentOutput) < 1.0E-2 ? BrakeState.BRAKING : BrakeState.FREE);
        climberMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    boolean hasStalledIntoBottom = false;

    /**
     * moves the climber down until it stall at the bottom. Press reset button to run again.
     */
    public synchronized void stallIntoBottom() {
        if (hasStalledIntoBottom) {
            setClimberMotor(0);
        } else {
            setClimberMotor(-0.3);
        }

        if (climberMotor.getStatorCurrent() > 12) {
            hasStalledIntoBottom = true;
            climberMotor.setSelectedSensorPosition(0);
            setClimberMotor(0);
        }
    }

    /**
     * Toggles the latch that is on the pivot arm.
     */
    public void toggleClaw() {
        isPaused = true;
        climbState = ClimbState.IDLE;
        setClawState(getClawState() == ClawState.UNLATCHED ? ClawState.LATCHED : ClawState.UNLATCHED);
    }

    /**
     * Toggles the pivot arm in and out. Also pauses the climber motor.
     */
    public void togglePivot() {
        isPaused = true;
        climbState = ClimbState.IDLE;
        setPivotState(getPivotState() == PivotState.INLINE ? PivotState.PIVOTED : PivotState.INLINE);
    }

    @Override
    public void selfTest() {

    }

    public @NotNull Climber.ClimbState getClimbState() {
        return climbState;
    }

    public boolean isPivotingArmLatched() {
        return isPivotingArmLatchedSwitchA() && isElevatorArmContactSwitchB();
    }

    public boolean isPivotingArmLatchedSwitchA() {
        return !pivotingArmLatchedSwitchA.get();
    }

    public boolean isElevatorArmContactSwitchB() {
        return !pivotingArmLatchedSwitchB.get();
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
        logData("Pivoting Arm Contact Switch A", isPivotingArmLatchedSwitchA());
        logData("Pivoting Arm Contact Switch B", isElevatorArmContactSwitchB());

        logData("Pivot Solenoid State", getPivotState().toString());
        logData("Latch Solenoid State", getClawState().toString());
        logData("Brake Solenoid State", getBrakeState().toString());

        logData("Gyro Pitch", RobotTracker.getInstance().getGyro().getPitch());
        logData("Gyro Pitch Velocity", gyroPitchVelocity);
        logData("Gyro Roll", RobotTracker.getInstance().getGyro().getRoll());
        logData("Gyro Roll Velocity", gyroRollVelocity);
        logData("Gyro Roll Acceleration", gyroRollAccel);

        logData("Climber Is Paused", isPaused);
        logData("Climber Is Step By Step", sensorClimb);
        logData("Current Climber State", climbState.toString());

        if (sensorClimb) {
            logData("Current Climber WaitCondition", climbState.positionClimb.waitCondition.apply(this));
        } else {
            logData("Current Climber WaitCondition", climbState.sensorClimb.waitCondition.apply(this));
        }
    }

    public void configCoast() {
        climberMotor.setNeutralMode(NeutralMode.Coast);
        climberMotor2.setNeutralMode(NeutralMode.Coast);
        setBrakeState(BrakeState.FREE);
    }

    public void configBrake() {
        climberMotor.setNeutralMode(NeutralMode.Coast);
        climberMotor2.setNeutralMode(NeutralMode.Coast);
        setBrakeState(BrakeState.BRAKING);
    }


    public boolean isPaused() {
        return isPaused;
    }


    @Override
    public void close() throws Exception {

    }
}
