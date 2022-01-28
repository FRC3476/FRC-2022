package frc.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
import frc.robot.Constants;
import frc.utility.Timer;
import frc.utility.controllers.LazyCANSparkMax;
import frc.utility.controllers.LazyTalonFX;
import frc.utility.controllers.LazyTalonSRX;

/**
 * Shooter class controls the shooter flywheel, feeder wheel, and variable hood Has motor control wrappers for setting velocity
 * and position for respective parts.
 * <p>
 * Can get speeds of motor and states / modes included in the class Has modes for HOMING and TESTING.
 * <p>
 * Has support for logging to Shuffleboard Communicates some shooting, testing, and homing states with BlinkinLED.
 */
public class Shooter extends AbstractSubsystem {

    // Talon500 Initialization
    private LazyTalonFX shooterWheelMaster;
    private LazyTalonFX shooterWheelSlave;
    private double desiredShooterSpeed;

    // 775Pro Initialization (TalonSRX may change to spark in the future)
    private LazyTalonSRX feederWheel;
    private double forceFeederOnTime;

    // NEO550 Initialization
    private LazyCANSparkMax hoodMotor;
    private SparkMaxPIDController hoodPID;
    private RelativeEncoder hoodRelativeEncoder;

    // REV Through Bore Encoder Initialization
    private DutyCycle hoodAbsoluteEncoder;

    // Home Switch Initialization
    private DigitalInput homeSwitch;

    // Homing initialization

    // currentPosition will be updated after each increment of .1 motor rotations
    // it will then be used to calculate the next targetPosition by incrementing it by .1
    private double currentHomingPosition;

    // startTime and currentTime will be used to check how long the homing is occurring
    // Negative 1 used for a check to see if homingStartTime has been initialized in HOMING case
    private double homingStartTime = -1;
    private double homingCurrentTime;

    // Declarations of Modes and States

    /**
     * Hood Position can either be obtained through the relative encoder on the NEO550 motor that controls the hood, or it can be
     * obtained through an absolute encoder which has a 1:1 ratio with the hood angle.
     */
    public enum HoodPositionMode {
        /**
         * Hood Position obtained through the absolute encoder which has a 1:1 ratio to the hood angle.
         */
        ABSOLUTE_ENCODER,

        /**
         * Hood Position obtained through relative encoder in the NEO550.
         */
        RELATIVE_TO_HOME
    }

    private HoodPositionMode hoodPositionMode;

    /**
     * Home Switch that is used for homing the relative encoder on the NEO550 that controls the hood.
     */
    public enum HomeSwitchState {
        /**
         * Home Switch is pressed.
         */
        PRESSED,

        /**
         * Home Switch is not pressed.
         */
        NOT_PRESSED
    }

    /**
     * Feeder Wheel does not have to be controlled accurately like the flywheel or hood; therefore, we have two states: ON and
     * OFF.
     */
    public enum FeederWheelState {
        /**
         * Feeder Wheel is ON, uses 100% of power it has.
         */
        ON,

        /**
         * Feeder Wheel is OFF.
         */
        OFF
    }

    private FeederWheelState feederWheelState;

    /**
     * Shooter can either be OFF, ON, HOMING, or TESTING.
     * <p>
     * OFF locks the motors ON allows commands to be sent to the motors HOMING homes the relative encoder on the hood TEST tests
     * the functionality of Shooter Flywheel, Feeder Wheel, and Hood. (Max and Min angles)
     * <p>
     * If HOMING or TESTING is occurring and there is a request to change the shooter state, the state change will be placed in a
     * 1 slot queue where it will wait for HOMING or TESTING to be finished before switching to requested state.
     */
    public enum ShooterState {
        /**
         * Will turn off all motors.
         */
        OFF,

        /**
         * Shooter is homing the hood.
         */
        HOMING,

        /**
         * Commands may be sent to motors.
         */
        ON,

        /**
         * Test state that will drive motors expected conditions.
         */
        TEST
    }

    private ShooterState shooterState = ShooterState.ON;

    /**
     * A 1 slot queue that holds a requested state change if the state could not be changed at the moment.
     * <p>
     * This will be used when a state is requested to be changed to while HOMING or TESTING is occurring.
     */
    public enum NextState {
        /**
         * Will turn off all motors.
         */
        OFF,

        /**
         * Shooter is homing the hood.
         */
        HOMING,

        /**
         * Commands may be sent to motors.
         */
        ON,

        /**
         * Test state that will drive motors expected conditions.
         */
        TEST
    }

    private NextState nextState;

    // The target hood angle
    private double desiredHoodAngle;

    // Test Pass or Fail Booleans, only records latest tests

    /**
     * Test Result of if the shooter flywheel accurately speeds up to a certain speed.
     */
    public boolean shooterMeetsExpectedSpeedTest;

    /**
     * Test Result of if the feeder wheel turns on and speeds up.
     */
    public boolean feederMeetsExpectedSpeedTest;

    /**
     * Test Result of if the hood is able to travel to the MINIMUM angle it should: 50 degrees from horizontal.
     */
    public boolean hoodMeetsMinimumAngleTest;

    /**
     * Test Result of if the hood is able to travel to the MAXIMUM angle it should: 90 degrees from the horizontal.
     */
    public boolean hoodMeetsMaximumAngleTest;

    /**
     * The current test being executed. Will be null if no test is being executed.
     */
    public enum CurrentTest {
        /**
         * Shooter is currently being tested.
         */
        SHOOTER,

        /**
         * Feeder is currently being tested.
         */
        FEEDER,

        /**
         * Hood is currently being tested to see if it can make it to the MINIMUM expected angle.
         */
        HOOD_MIN_ANGLE,

        /**
         * Hood is currently being tested to see if it can make it to the MAXIMUM expected angle.
         */
        HOOD_MAX_ANGLE,

        /**
         * All tests are finished and testing operations are being wrapped up.
         */
        TESTS_FINISHED
    }

    private CurrentTest currentTest;

    // Test Timing Constants
    private double testStartTime = -1;
    private double testCurrentTime;

    // Singleton Setup
    private static Shooter instance = new Shooter();

    /**
     * returns the singleton instance of Shooter.
     */
    public static Shooter getInstance() {
        return instance;
    }

    // Private constructor for singleton
    private Shooter() {
        // TODO: May have to invert direction of motors
        // Sets update method's iteration
        super(Constants.SHOOTER_PERIOD_MS, Constants.SHOOTER_PERIODS_PER_LOG);

        // Sets hood position mode, can be either using absolute encoder or be relative to home switch
        hoodPositionMode = HoodPositionMode.ABSOLUTE_ENCODER;

        // Sets CAN IDs to each component
        shooterWheelMaster = new LazyTalonFX(Constants.SHOOTER_WHEEL_CAN_MASTER_ID);
        shooterWheelSlave = new LazyTalonFX(Constants.SHOOTER_WHEEL_CAN_SLAVE_ID);
        feederWheel = new LazyTalonSRX(Constants.FEEDER_WHEEL_CAN_ID);
        hoodMotor = new LazyCANSparkMax(Constants.HOOD_MOTOR_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        hoodAbsoluteEncoder = new DutyCycle(new DigitalInput(Constants.HOOD_ENCODER_DIO_ID));
        hoodRelativeEncoder = hoodMotor.getEncoder();
        hoodRelativeEncoder.setPositionConversionFactor(Constants.HOOD_DEGREES_PER_MOTOR_ROTATION);
        homeSwitch = new DigitalInput(Constants.HOOD_HOME_SWITCH_DIO_ID);

        // Sets one of the shooter motors inverted
        shooterWheelSlave.setInverted(true);

        configPID();
    }

    private void configPID() {
        // Second Falcon Follows Speed Of Main Falcon
        shooterWheelSlave.follow(shooterWheelMaster);

        // Configure PID Constants and current limit
        shooterWheelMaster.config_kP(0, Constants.SHOOTER_P, Constants.SHOOTER_PID_TIMEOUT_MS);
        shooterWheelMaster.config_kI(0, Constants.SHOOTER_I, Constants.SHOOTER_PID_TIMEOUT_MS);
        shooterWheelMaster.config_kD(0, Constants.SHOOTER_D, Constants.SHOOTER_PID_TIMEOUT_MS);
        shooterWheelMaster.config_kF(0, Constants.SHOOTER_F, Constants.SHOOTER_PID_TIMEOUT_MS);
        shooterWheelMaster.config_IntegralZone(0, Constants.SHOOTER_I_ZONE, Constants.SHOOTER_PID_TIMEOUT_MS);

        shooterWheelMaster.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.SHOOTER_CURRENT_LIMIT,
                Constants.SHOOTER_TRIGGER_THRESHOLD_CURRENT, Constants.SHOOTER_TRIGGER_THRESHOLD_TIME));


        // This PID setup for 775pro will not be used
        feederWheel.config_kP(0, Constants.FEEDER_WHEEL_P, Constants.SHOOTER_PID_TIMEOUT_MS);
        feederWheel.config_kI(0, Constants.FEEDER_WHEEL_I, Constants.SHOOTER_PID_TIMEOUT_MS);
        feederWheel.config_kD(0, Constants.FEEDER_WHEEL_D, Constants.SHOOTER_PID_TIMEOUT_MS);
        feederWheel.config_kF(0, Constants.FEEDER_WHEEL_F, Constants.SHOOTER_PID_TIMEOUT_MS);
        feederWheel.config_IntegralZone(0, Constants.FEEDER_WHEEL_I_ZONE, Constants.SHOOTER_PID_TIMEOUT_MS);

        shooterWheelMaster.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.FEEDER_CURRENT_LIMIT,
                Constants.FEEDER_TRIGGER_THRESHOLD_CURRENT, Constants.FEEDER_TRIGGER_THRESHOLD_TIME));

        hoodPID = hoodMotor.getPIDController();
        hoodPID.setP(Constants.HOOD_P, 0);
        hoodPID.setI(Constants.HOOD_I, 0);
        hoodPID.setD(Constants.HOOD_D, 0);
        hoodPID.setFF(Constants.HOOD_F, 0);
        hoodPID.setIZone(Constants.HOOD_I_ZONE);
        hoodMotor.setSmartCurrentLimit(Constants.HOOD_CURRENT_LIMIT_AMPS);
    }

    /**
     * Gets the hood angle. Expected range will be from 50-90, but it may be outside this range.
     * <p>
     * Will get the hood angle using the currently selected HoodPosition mode - the encoder that will be used to obtain the angle
     * of the hood.
     * <p>
     * If hood is outside expected range, warning will be sent to drive station.
     */
    public double getHoodAngle() {
        double hoodAngle;

        // If using Absolute Encoder
        if (hoodPositionMode == HoodPositionMode.ABSOLUTE_ENCODER) {
            // Adds and offset to correct angle of the encoder
            hoodAngle = getHoodAbsoluteEncoderValue() + Constants.HOOD_ABSOLUTE_ENCODER_OFFSET;

            // Checks if Absolute Encoder is reading outside of expected range
            if (hoodAngle > 90.5 || hoodAngle < 49.5) {
                DriverStation.reportWarning("Hood position reading values over MAX expected.\n" +
                        "Should switch to home relative", false);
            }
        } else {
            // If using Relative Encoder
            hoodAngle = hoodRelativeEncoder.getPosition();
        }

        return hoodAngle;
    }

    // Raw degree measurement from Absolute Encoder, May need an offset
    private double getHoodAbsoluteEncoderValue() {
        return hoodAbsoluteEncoder.getOutput() * 360;
    }

    /**
     * Sets ShooterState to HOMING.
     * <p>
     * Motors will not accept inputs and HOMING process for hood will occur Blinkin LED will be changed to denote that HOMING is
     * occurring.
     * <p>
     * If a request to change state is occurring while robot is HOMING, state will be put into a 1 slot queue where it will be
     * changed to after HOMING has completed.
     */
    public void homeHood() {
        // Checks to make sure that robot is not currently HOMING or TESTING
        if (shooterState == ShooterState.HOMING || shooterState == ShooterState.TEST) {
            nextState = nextState.HOMING;
        } else {
            shooterState = ShooterState.HOMING;
        }
    }

    /**
     * Toggles between methods to get hood positions.
     * <p>
     * Includes two options, ABSOLUTE_ENCODER and RELATIVE_TO_HOME.
     * <p>
     * ABSOLUTE_ENCODER uses the absolute encoder that has a 1:1 ratio with the hood angle.
     * <p>
     * RELATIVE_TO_HOME uses the built in NEO550 relative encoder which is not 1:1 with hood angle.
     */
    public void toggleHoodPositionMode() {
        // Checks current mode and switches to the alternate mode
        if (getHoodPositionMode() == HoodPositionMode.ABSOLUTE_ENCODER) {
            hoodPositionMode = HoodPositionMode.RELATIVE_TO_HOME;
            homeHood();
        } else {
            hoodPositionMode = HoodPositionMode.ABSOLUTE_ENCODER;
        }
    }

    /**
     * Returns if the Hood Position is being obtained through an absolute or relative encoder.
     */
    public HoodPositionMode getHoodPositionMode() {
        return hoodPositionMode;
    }

    /**
     * Sets Speed of Shooter FlywheelWheel. Parameter should be in RPM.
     */
    public void setShooterSpeed(double desiredShooterSpeed) {
        this.desiredShooterSpeed = desiredShooterSpeed;
    }

    /**
     * Enables Feeder Wheel. Will run at MAXIMUM speed.
     */
    public void enableFeederWheel() {
        feederWheelState = FeederWheelState.ON;
    }

    /**
     * Stops Feeder Wheel.
     */
    public void disableFeederWheel() {
        feederWheelState = FeederWheelState.OFF;
    }

    /**
     * Gets the state of the Feeder Wheel.
     * <p>
     * Can either be ON or OFF.
     */
    public FeederWheelState getFeederWheelState() {
        return feederWheelState;
    }

    /**
     * Updates the state of home switch.
     * <p>
     * Can either be PRESSED or NOT_PRESSED.
     */
    public HomeSwitchState getHomeSwitchState() {
        return homeSwitch.get() ? HomeSwitchState.PRESSED : HomeSwitchState.NOT_PRESSED;
    }

    /**
     * Sets the Hood Position to a desired angle between 50 and 90.
     */
    public void setHoodPosition(double desiredAngle) {
        // Preform Bounds checking between MAX and MIN range
        if (desiredAngle < 50) {
            desiredAngle = 50;
        }

        if (desiredAngle > 90) {
            desiredAngle = 90;
        }
        desiredHoodAngle = desiredAngle;
    }

    /**
     * Gets the angular speed of the shooter flywheel in RPM.
     */
    public double getShooterRPM() {
        // Convert Falcon 500 encoder units for velocity into RPM
        return shooterWheelMaster.getSelectedSensorVelocity() * Constants.FALCON_UNIT_CONVERSION_FOR_RELATIVE_ENCODER;
    }

    /**
     * Returns desired/commanded shooter speed.
     * <p>
     * Specifically, this is the setpoint for the shooter flywheel PID controller.
     */
    public double getDesiredShooterSpeed() {
        return desiredShooterSpeed;
    }

    /**
     * Checks if shooter is at target speed within a configurable allowed error.
     */
    public boolean isShooterAtTargetSpeed() {
        return getShooterRPM() > Math.abs(getDesiredShooterSpeed() - (Constants.ALLOWED_SHOOTER_SPEED_ERROR_RPM / 2.0)) &&
                getShooterRPM() < Math.abs(getDesiredShooterSpeed() + (Constants.ALLOWED_SHOOTER_SPEED_ERROR_RPM / 2.0));
    }

    /**
     * Sets shooter state to OFF, will lock motors.
     */
    public void turnShooterOFF() {
        // Checks to make sure that HOMING or TEST is not occurring
        if (shooterState == ShooterState.HOMING || shooterState == ShooterState.TEST) {
            nextState = nextState.OFF;
        } else {
            shooterState = ShooterState.HOMING.OFF;
        }
    }

    /**
     * Turns shooter ON
     * <p>
     * Allows motors to receive commands
     */
    public void turnShooterON() {
        // Checks to make sure that HOMING or TEST is not occurring
        if (shooterState == ShooterState.HOMING || shooterState == ShooterState.TEST) {
            nextState = NextState.ON;
        } else {
            shooterState = ShooterState.ON;
        }
    }

    /**
     * Checks if Hood is at the target angle within a configurable allowed error.
     */
    public boolean isHoodAtTargetAngle() {
        return getHoodAngle() > (Math.abs(getDesiredHoodAngle() - Constants.ALLOWED_HOOD_ANGLE_ERROR) / 2.0) &&
                getHoodAngle() < (Math.abs(getDesiredHoodAngle() + Constants.ALLOWED_HOOD_ANGLE_ERROR) / 2.0);
    }

    /**
     * Returns desired/commanded hood angle.
     */
    public double getDesiredHoodAngle() {
        return desiredHoodAngle;
    }

    /**
     * Returns current Shooter State.
     * <p>
     * Can be OFF, ON, HOMING, or TEST.
     */
    public ShooterState getShooterState() {
        return shooterState;
    }

    /**
     * Returns queued state for states that have been requested but could not be changed immediately.
     *
     * @return returns queued state, may be null if there is no queued state.
     */
    public NextState getNextState() {
        return nextState;
    }

    // Changes shooter state to nextValue, will be called
    private void setNextStateToCurrentState() {
        // Changes shooter state to nextValue
        if (nextState == NextState.ON) {
            shooterState = shooterState.ON;
        } else if (nextState == NextState.OFF) {
            shooterState = shooterState.OFF;
        } else if (nextState == NextState.HOMING) {
            shooterState = shooterState.HOMING;
        } else if (nextState == NextState.TEST) {
            shooterState = shooterState.TEST;
        } else {
            // Defaults to ON if next state is not specified
            shooterState = shooterState.ON;
        }

        // Resets nextState
        nextState = null;
    }

    // Sets Blinkin LED to color when robot is OFF
    private void setLedForOffMode() {
        BlinkinLED.getInstance().setColor(Constants.LED_SHOOTER_OFF);
    }

    // Sets LED different colors, depends on if flywheel is up to speed and if hood is in correct position
    private void setLedForOnMode() {

        // Sets LED different colors for different shooter scenarios
        if (isShooterAtTargetSpeed() == false) {
            // If Shooter is not at the target speed, LED will display this color
            BlinkinLED.getInstance().setColor(Constants.LED_FLYWHEEL_APPROACHING_DESIRED_SPEED);
        } else if (isHoodAtTargetAngle() == false) {
            // If Shooter is not at the target angle, but it is at the target speed, LED will display this color
            BlinkinLED.getInstance().setColor(Constants.LED_HOOD_APPROACHING_DESIRED_POSITION);
        } else {
            // If both hood and flywheel are ready for shooting, LED will display this color
            BlinkinLED.getInstance().setColor(Constants.LED_SHOOTER_READY_TO_SHOOT);
        }
    }

    // Sets BlinkinLED to color when robot is in homing mode
    private void setLedForHomingMode() {
        BlinkinLED.getInstance().setColor(Constants.LED_HOOD_HOMING_IN_PROGRESS);
    }

    // Sets BlinkinLED to color when robot is in test mode
    private void setLedForTestMode() {
        BlinkinLED.getInstance().setColor(Constants.LED_TEST_IN_PROGRESS);
    }

    /**
     * Gets the currently running test, will be null if no test is running.
     */
    public CurrentTest getCurrentTest() {
        return currentTest;
    }

    // Checks to see if current test is taking longer than expected
    private boolean doesCurrentTestExceedTimeLimit() {
        // Checks to see if difference between current test time and start of test time is greater than allowed test time
        return (testCurrentTime - testStartTime) > Constants.MAX_INDIVIDUAL_TEST_TIME_SEC;
    }

    /**
     * Update Method for Shooter.
     * <p>
     * Updates every 20ms.
     * <p>
     * Controls the actions that will be taken for each individual robot state.
     * <p>
     * OFF - locks motors.
     * <p>
     * ON - allows commands to be sent to motors.
     * <p>
     * HOMING - Enacts homing procedures.
     * <p>
     * TEST - Tests Shooter Flywheel, Feeder wheel, and hood.
     * <p>
     * Also controls BlinkinLED states.
     */
    @Override
    public void update() {
        // Switch statement only allows certain code to be run for specific states of the robot
        switch (shooterState) {
            case OFF:
                // Will lock the motors if Shooter State is OFF
                setShooterSpeed(0);
                disableFeederWheel();

                // Sets hood to lowest possible position
                setHoodPosition(50);

                // Sets LED for OFF mode
                setLedForOffMode();

                break;

            case ON:
                // Sets shooter motor to desired shooter speed
                shooterWheelMaster.set(ControlMode.Velocity, desiredShooterSpeed);

                // Sets Motor to travel to desired hood angle
                hoodPID.setReference((desiredHoodAngle - getHoodAngle()), CANSparkMax.ControlType.kPosition);

                // Checks to see if feeder wheel is enabled, if hoodMotor had finished moving, and if shooterWheel is at target
                // speed
                if ((feederWheelState == FeederWheelState.ON) && Math.abs(hoodRelativeEncoder.getVelocity()) < 1.0e-3 &&
                        isShooterAtTargetSpeed()) {

                    // Set Feeder wheel to MAX speed
                    feederWheel.set(ControlMode.PercentOutput, 1);
                    forceFeederOnTime = Timer.getFPGATimestamp() + 0.5;
                } else {

                    // Turn OFF Feeder Wheel if feederWheel has not been on in half a second
                    if (Timer.getFPGATimestamp() > forceFeederOnTime) {
                        feederWheel.set(ControlMode.PercentOutput, 0);
                    }
                }

                // Sets Blinkin LED different colors for different flywheel and hood states
                setLedForOnMode();

                break;

            case HOMING:
                // Does homing sequence if state is set to HOMING
                // Will turn motor towards home switch until home switch is enabled

                // Checks if homing has started yet
                if (homingStartTime == -1) {
                    // Records start of homing time, with current time being the same as the start time
                    homingStartTime = Timer.getFPGATimestamp();
                    homingCurrentTime = homingStartTime;

                    // Sends Homing start message to console
                    System.out.println("Homing Starting At: " + homingStartTime);
                }

                // Executes this if home switch is not pressed
                if (getHomeSwitchState() == HomeSwitchState.NOT_PRESSED) {
                    // Gets current time
                    homingCurrentTime = Timer.getFPGATimestamp();

                    // Gets current encoder value
                    currentHomingPosition = hoodRelativeEncoder.getPosition();

                    // Runs Motor at .05 Amps while home switch is not pressed
                    hoodPID.setReference(Constants.HOMING_MOTOR_CURRENT_AMPS, CANSparkMax.ControlType.kCurrent);
                } else {
                    // If home switch is pressed
                    // Sets current to zero if home switch is pressed
                    hoodPID.setReference(0, CANSparkMax.ControlType.kCurrent);

                    // Sets the relative encoder reference to the position of the home switch when Home switch is pressed
                    hoodRelativeEncoder.setPosition(90);

                    // Turns homing off and sets it to the next queued state, ON if no state queued
                    setNextStateToCurrentState();

                    // Sends Homing start message to console
                    System.out.println("Homing Finished Successfully At: " + homingCurrentTime);

                    // Resets Homing Start Time
                    homingStartTime = -1;
                }

                // Executes this if homing has been going on for the MAX allotted time
                if (homingCurrentTime - homingStartTime > Constants.MAX_HOMING_TIME_S) {
                    // Sets current to zero if max time is exceeded
                    hoodPID.setReference(0, CANSparkMax.ControlType.kCurrent);

                    DriverStation.reportWarning("Homing has taken longer than MAX expected time; homing has been stopped",
                            false);

                    // Turns homing off and sets it to the next queued state, ON if no state queued
                    setNextStateToCurrentState();

                    // Sends Homing start message to console
                    System.out.println("Homing Failed At: " + homingCurrentTime);

                    // Resets Homing Start Time
                    homingStartTime = -1;
                }

                // Sets LED for Homing states
                setLedForHomingMode();
                break;

            case TEST:

                // Initialization of test
                if (testStartTime == -1) {
                    // Sets start time and current time
                    testStartTime = Timer.getFPGATimestamp();
                    testCurrentTime = testStartTime;

                    // Resets last test booleans
                    shooterMeetsExpectedSpeedTest = false;
                    feederMeetsExpectedSpeedTest = false;
                    hoodMeetsMinimumAngleTest = false;
                    hoodMeetsMaximumAngleTest = false;

                    // Sets shooter to first test
                    currentTest = CurrentTest.SHOOTER;

                    // Prints out start of test time
                    System.out.println("Shooter Test Starting At: " + testCurrentTime);
                }

                // Runs specified test if current test equals that test

                // Tests Shooter Flywheel
                if (currentTest == CurrentTest.SHOOTER) {
                    // Sets shooter speed to test speed
                    setShooterSpeed(Constants.SHOOTER_TEST_SPEED);

                    if (isShooterAtTargetSpeed() == true) {
                        // Stops shooter
                        setShooterSpeed(0);

                        // Sets test boolean to true
                        shooterMeetsExpectedSpeedTest = true;

                        // Sets next test to feeder
                        currentTest = CurrentTest.FEEDER;

                        // Resets start time for next test
                        testStartTime = Timer.getFPGATimestamp();

                        // Prints out result of test
                        System.out.println("Shooter Flywheel Passed Speed Up Test At: " + testCurrentTime);
                    }

                    if (doesCurrentTestExceedTimeLimit()) {
                        // Stops shooter
                        setShooterSpeed(0);

                        // Sets test boolean to true
                        shooterMeetsExpectedSpeedTest = false;

                        // Sets next test to feeder
                        currentTest = CurrentTest.FEEDER;

                        // Resets start time for next test
                        testStartTime = Timer.getFPGATimestamp();

                        // Prints out result of test
                        System.out.println("Shooter Flywheel Failed Speed Up Test At: " + testCurrentTime);

                        DriverStation.reportWarning("Shooter Flywheel Failed Speed Up Test At: " + testCurrentTime, false);
                    }
                }

                // Tests Feeder Wheel
                if (currentTest == CurrentTest.FEEDER) {
                    // Enables Feeder Wheel
                    feederWheel.set(ControlMode.PercentOutput, 1);

                    // Checks to see if feederWheel is receiving proper current
                    if (feederWheel.getSupplyCurrent() > Constants.FEEDER_PASSING_TEST_CURRENT) {
                        // Disables Feeder Wheel
                        feederWheel.set(ControlMode.PercentOutput, 0);

                        // Sets test boolean to false
                        feederMeetsExpectedSpeedTest = true;

                        // Sets next test to hood MAX angle
                        currentTest = CurrentTest.HOOD_MAX_ANGLE;

                        // Resets start time for next test
                        testStartTime = Timer.getFPGATimestamp();

                        // Prints out result of test
                        System.out.println("Shooter Feeder Passed Speed Up Test At: " + testCurrentTime);
                    }

                    if (doesCurrentTestExceedTimeLimit()) {
                        // Disables Feeder Wheel
                        feederWheel.set(ControlMode.PercentOutput, 0);

                        // Sets test boolean to false
                        feederMeetsExpectedSpeedTest = false;

                        // Sets next test to hood MAX angle
                        currentTest = CurrentTest.HOOD_MAX_ANGLE;

                        // Resets start time for next test
                        testStartTime = Timer.getFPGATimestamp();

                        // Prints out result of test
                        System.out.println("Shooter Feeder Failed Speed Up Test At: " + testCurrentTime);

                        DriverStation.reportWarning("Shooter Feeder Failed Speed Up Test At: " + testCurrentTime, false);
                    }
                }

                // Tests Maximum hood angle
                if (currentTest == CurrentTest.HOOD_MAX_ANGLE) {
                    // Sets hood to MAX position
                    setHoodPosition(90);

                    if (isHoodAtTargetAngle()) {
                        // Sets test boolean to true
                        hoodMeetsMaximumAngleTest = true;

                        // Sets next test to hood MIN angle
                        currentTest = CurrentTest.HOOD_MIN_ANGLE;

                        // Resets start time for next test
                        testStartTime = Timer.getFPGATimestamp();

                        // Prints out result of test
                        System.out.println("Shooter Hood Reached MAXIMUM Angle At: " + testCurrentTime);
                    }

                    if (doesCurrentTestExceedTimeLimit()) {
                        // Sets test boolean to false
                        hoodMeetsMaximumAngleTest = false;

                        // Sets next test to hood MIN angle
                        currentTest = CurrentTest.HOOD_MIN_ANGLE;

                        // Resets start time for next test
                        testStartTime = Timer.getFPGATimestamp();

                        // Prints out result of test
                        System.out.println("Shooter Hood Failed To Reach MAXIMUM Angle At: " + testCurrentTime);

                        DriverStation.reportWarning("Shooter Hood Failed To Reach MAXIMUM Angle At: " + testCurrentTime, false);
                    }
                }

                // Tests MINIMUM Hood Angle
                if (currentTest == CurrentTest.HOOD_MIN_ANGLE) {
                    // Sets hood to MIN position
                    setHoodPosition(50);

                    if (isHoodAtTargetAngle()) {
                        // Sets test boolean to true
                        hoodMeetsMinimumAngleTest = true;

                        // Allows for test wrap up to occur
                        currentTest = CurrentTest.TESTS_FINISHED;

                        // Resets start time for next test
                        testStartTime = Timer.getFPGATimestamp();

                        // Prints out result of test
                        System.out.println("Shooter Hood Reached MINIMUM Angle At: " + testCurrentTime);
                    }

                    if (doesCurrentTestExceedTimeLimit()) {
                        // Sets test boolean to false
                        hoodMeetsMinimumAngleTest = false;

                        // Sets next test to hood MIN angle
                        currentTest = CurrentTest.TESTS_FINISHED;

                        // Resets start time for next test
                        testStartTime = Timer.getFPGATimestamp();

                        // Prints out result of test
                        System.out.println("Shooter Hood Failed To Reach MINIMUM Angle At: " + testCurrentTime);

                        DriverStation.reportWarning("Shooter Hood Failed To Raech MINIMUM Angle At " + testCurrentTime, false);
                    }
                }

                // Wraps up testing
                if (currentTest == CurrentTest.TESTS_FINISHED) {
                    // Updates Current Test Time
                    testCurrentTime = Timer.getFPGATimestamp();

                    // Turns TESTING off and sets it to the next queued state, ON if no state queued
                    setNextStateToCurrentState();

                    // Fully resets testing start time
                    testStartTime = -1;

                    // Resets currentTest
                    currentTest = null;

                    // Prints out alert that tests are finished
                    System.out.println("All Shooter Tests Completed At: " + testCurrentTime);
                }
                // Updates Current Test Time
                testCurrentTime = Timer.getFPGATimestamp();

                // Sets LED for testing states
                setLedForTestMode();

                break;
        }
    }

    /**
     * Sets robot state to TEST mode.
     * <p>
     * Will activate self test that diagnoses functionality of the Shooter Flywheel, Feeder Wheel, and Hood.
     * <p>
     * Test results will be printed to the console.
     * <p>
     * Failed tests will send warnings to the drive station.
     */
    @Override
    public void selfTest() {
        // Checks to make sure that robot is not currently HOMING or TESTING
        if (shooterState == ShooterState.HOMING || shooterState == ShooterState.TEST) {
            nextState = nextState.TEST;
        } else {
            shooterState = ShooterState.TEST;
        }
    }

    /**
     * Logs variety of values and states to ShuffleBoard.
     * <p>
     * The logging period is every 60ms.
     * <p>
     * Refer to initialization in Shooter class to find specifics of what is logged.
     */
    @Override
    public void logData() {
        logData("Shooter Flywheel Speed", getShooterRPM());
        logData("Hood Angle", getHoodAngle());
        logData("Desired Shooter Speed", getDesiredShooterSpeed());
        logData("Desired Hood Angle", getDesiredHoodAngle());
        logData("Feeder Wheel State", getFeederWheelState());
        logData("Home Switch State", getHomeSwitchState());
        logData("Hood Positioning Mode", getHoodPositionMode());
        logData("Is Hood at Target Angle?", isHoodAtTargetAngle());
        logData("Is Shooter at Target Speed?", isShooterAtTargetSpeed());
        logData("Shooter State", getShooterState());
        logData("Shooter Flywheel Speed Error", getDesiredShooterSpeed() - getShooterRPM());
        logData("Hood Position Error", getDesiredHoodAngle() - getHoodAngle());
        logData("Current Test", getCurrentTest());
    }

    /** Closing of Shooter motors is not supported. */
    @Override
    public void close() throws UnsupportedOperationException {
        throw new UnsupportedOperationException("Can not close this object");
    }
}
