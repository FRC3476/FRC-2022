package frc.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.utility.OrangeUtility;
import frc.utility.Timer;
import frc.utility.controllers.LazyCANSparkMax;
import frc.utility.controllers.LazyTalonFX;
import org.jetbrains.annotations.NotNull;

/**
 * Shooter class controls the shooter flywheel, feeder wheel, and variable hood Has motor control wrappers for setting velocity
 * and position for respective parts.
 * <p>
 * Can get speeds of motor and states / modes included in the class Has modes for HOMING and TESTING.
 * <p>
 * Has support for logging to Shuffleboard Communicates some shooting, testing, and homing states with BlinkinLED.
 */

public final class Shooter extends AbstractSubsystem {

    // Talon500 Initialization

    // Shooter Flywheel
    private final LazyTalonFX shooterWheelMaster;
    private final LazyTalonFX shooterWheelSlave;
    private double desiredShooterSpeed;

    // Feeder
    private final LazyTalonFX feederWheel;
    private double forceFeederOnTime;

    // NEO550 Initialization

    // Hood
    private final LazyCANSparkMax hoodMotor;
    private SparkMaxPIDController hoodPID;
    private final RelativeEncoder hoodRelativeEncoder;

    // REV Through Bore Encoder Initialization
    private final CANCoder hoodAbsoluteEncoder;

    // Home Switch Initialization
    private final DigitalInput homeSwitch;

    // Homing initialization

    // startTime and currentTime will be used to check how long the homing is occurring
    // Negative 1 used for a check to see if homingStartTime has been initialized in HOMING case
    private double homingStartTime = -1;

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
     * Feeder Wheel does not have to be controlled accurately like the flywheel or hood; therefore, we have three states: ON and
     * OFF.
     */
    public enum FeederWheelState {
        /**
         * Feeder Wheel is running forwards, uses 100% of power it has.
         */
        FORWARD,

        /**
         * Feeder Wheel is OFF.
         */
        OFF,
    }

    private FeederWheelState feederWheelState = FeederWheelState.OFF;

    /**
     * Shooter can either be OFF, ON, HOMING, or TESTING.
     * <p>
     * OFF, turns off the motors
     * <p>
     * ON allows commands to be sent to the motors
     * <p>
     * HOMING homes the relative encoder on the hood
     * <p>
     * TEST tests the functionality of Shooter Flywheel, Feeder Wheel, and Hood. (Max and Min angles)
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
         * Commands may be sent to the motors.
         */
        ON,

        /**
         * Test state that will drive motors expected conditions.
         */
        TEST
    }

    private ShooterState shooterState = ShooterState.OFF;

    /**
     * A 1 slot queue that holds a requested state change if the state could not be changed at the moment.
     * <p>
     * This will be used when a state is requested to be changed to while HOMING or TESTING is occurring.
     */

    private @NotNull ShooterState nextState = ShooterState.OFF;

    // The target hood angle
    private double desiredHoodAngle;

    // Singleton Setup

    private static final Shooter instance = new Shooter();

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
        feederWheel = new LazyTalonFX(Constants.FEEDER_WHEEL_CAN_ID);
        hoodMotor = new LazyCANSparkMax(Constants.HOOD_MOTOR_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        hoodAbsoluteEncoder = new CANCoder(Constants.HOOD_ABSOLUTE_ENCODER_CAN_ID);
        hoodRelativeEncoder = hoodMotor.getEncoder();
        hoodRelativeEncoder.setPositionConversionFactor(Constants.HOOD_DEGREES_PER_MOTOR_ROTATION);
        homeSwitch = new DigitalInput(Constants.HOOD_HOME_SWITCH_DIO_ID);

        // Sets one of the shooter motors inverted
        shooterWheelSlave.setInverted(true);
        shooterWheelMaster.setInverted(false);

        configPID();
    }

    private void configPID() {
        // Second Falcon Follows Speed Of Main Falcon
        shooterWheelSlave.follow(shooterWheelMaster);

        // Configure PID Constants and current limit
        shooterWheelMaster.config_kP(0, Constants.SHOOTER_P);
        shooterWheelMaster.config_kI(0, Constants.SHOOTER_I);
        shooterWheelMaster.config_kD(0, Constants.SHOOTER_D);
        shooterWheelMaster.config_kF(0, Constants.SHOOTER_F);
        shooterWheelMaster.config_IntegralZone(0, Constants.SHOOTER_I_ZONE);

        shooterWheelMaster.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.SHOOTER_CURRENT_LIMIT,
                Constants.SHOOTER_TRIGGER_THRESHOLD_CURRENT, Constants.SHOOTER_TRIGGER_THRESHOLD_TIME));
        shooterWheelSlave.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.SHOOTER_CURRENT_LIMIT,
                Constants.SHOOTER_TRIGGER_THRESHOLD_CURRENT, Constants.SHOOTER_TRIGGER_THRESHOLD_TIME));

        shooterWheelMaster.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);

        // Turns off brake mode for shooter flywheel
        shooterWheelMaster.setNeutralMode(NeutralMode.Coast);
        shooterWheelSlave.setNeutralMode(NeutralMode.Coast);


        // This PID setup for 775pro will not be used
        feederWheel.config_kP(0, Constants.FEEDER_WHEEL_P);
        feederWheel.config_kI(0, Constants.FEEDER_WHEEL_I);
        feederWheel.config_kD(0, Constants.FEEDER_WHEEL_D);
        feederWheel.config_kF(0, Constants.FEEDER_WHEEL_F);
        feederWheel.config_IntegralZone(0, Constants.FEEDER_WHEEL_I_ZONE);

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

            // Checks if Absolute Encoder is reading outside expected range
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

    // Raw degree measurement from Absolute Encoder, The angle may need an offset
    private double getHoodAbsoluteEncoderValue() {
        return hoodAbsoluteEncoder.getAbsolutePosition() * 360;
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
            shooterState = ShooterState.HOMING;
    }

    /**
     * Allows setting of Hood Position Mode
     * <p>
     * Includes two options, ABSOLUTE_ENCODER and RELATIVE_TO_HOME.
     * <p>
     * ABSOLUTE_ENCODER uses the absolute encoder that has a 1:1 ratio with the hood angle.
     * <p>
     * RELATIVE_TO_HOME uses the built-in NEO550 relative encoder which is not 1:1 with hood angle.
     * <p>
     * Switching to relative mode expects hood to be homed already.
     */
    public void setHoodPositionMode(HoodPositionMode hoodPositionMode) {
        this.hoodPositionMode = hoodPositionMode;
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
        if (desiredShooterSpeed == 0) {
            if (shooterState == ShooterState.HOMING || shooterState == ShooterState.TEST) {
                nextState = ShooterState.OFF;
            } else {
                shooterState = ShooterState.OFF;
            }
        } else {
            if (shooterState == ShooterState.HOMING || shooterState == ShooterState.TEST) {
                nextState = ShooterState.ON;
            } else {
                shooterState = ShooterState.ON;
            }
        }
    }

    /**
     * Turns the shooter feeder-wheel on/off
     *
     * @param shoot true to turn on, false to turn off
     */
    public void setFiring(boolean shoot) {
        if (shoot) {
            feederWheelState = FeederWheelState.FORWARD;
        } else {
            feederWheelState = FeederWheelState.OFF;
        }
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
        return Math.abs(getShooterRPM() - getDesiredShooterSpeed()) < Constants.ALLOWED_SHOOTER_SPEED_ERROR_RPM;
    }

    /**
     * Checks if Hood is at the target angle within a configurable allowed error.
     */
    public boolean isHoodAtTargetAngle() {
        return Math.abs(getHoodAngle() - getDesiredHoodAngle()) < Constants.ALLOWED_HOOD_ANGLE_ERROR;
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
     * @return returns queued state
     */
    public @NotNull ShooterState getNextState() {
        return nextState;
    }

    // Sets LED different colors, depends on if flywheel is up to speed and if hood is in correct position
    private void setLedForOnMode() {

        // Sets LED different colors for different shooter scenarios
        if (!isShooterAtTargetSpeed()) {
            // If Shooter is not at the target speed, LED will display this color
            BlinkinLED.getInstance().setColor(Constants.LED_FLYWHEEL_APPROACHING_DESIRED_SPEED);
        } else if (!isHoodAtTargetAngle()) {
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

    private void moveHoodMotor() {
        hoodPID.setReference((desiredHoodAngle - getHoodAngle() + hoodRelativeEncoder.getPosition()),
                CANSparkMax.ControlType.kPosition);
    }

    /**
     * Update Method for Shooter.
     * <p>
     * Updates every 20ms.
     * <p>
     * Controls the actions that will be taken for each individual robot state.
     * <p>
     * OFF - turns off motors.
     * <p>
     * ON - allows commands to be sent to the motors.
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
                setShooterSpeed(0);
                setHoodPosition(50); // Sets hood to the lowest possible position
                feederWheel.set(ControlMode.PercentOutput, 0);

                break;

            case ON:
                shooterWheelMaster.set(ControlMode.Velocity, desiredShooterSpeed); // Sets shooter motor to desired shooter speed

                moveHoodMotor(); // Sets Motor to travel to desired hood angle

                // Checks to see if feeder wheel is enabled forward, if hoodMotor had finished moving, and if shooterWheel
                // is at target speed
                if ((feederWheelState == FeederWheelState.FORWARD) && isHoodStopped() && isShooterAtTargetSpeed()) {

                    // Set Feeder wheel to MAX speed
                    feederWheel.set(ControlMode.PercentOutput, Constants.FEEDER_WHEEL_SPEED);
                    forceFeederOnTime = Timer.getFPGATimestamp() + Constants.FEEDER_CHANGE_STATE_DELAY_SEC;
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
                //noinspection FloatingPointEquality
                if (homingStartTime == -1) {
                    // Records start of homing time, with current time being the same as the start time
                    homingStartTime = Timer.getFPGATimestamp();

                    // Sends Homing start message to console
                    System.out.println("Homing Starting At: " + homingStartTime);
                }

                // Executes this if home switch is pressed
                if (getHomeSwitchState() == HomeSwitchState.PRESSED) {

                    // Sets the relative encoder reference to the position of the home switch when Home switch is pressed
                    hoodRelativeEncoder.setPosition(90);
                    // Sets current to zero if home switch is pressed
                    hoodPID.setReference(0, CANSparkMax.ControlType.kDutyCycle);


                    // Turns homing off and sets it to the next queued state, ON if no state queued
                    shooterState = nextState;

                    // Sends Homing start message to console
                    System.out.println("Homing Finished Successfully At: " + Timer.getFPGATimestamp());

                    // Resets Homing Start Time
                    homingStartTime = -1;
                } else {

                    // Runs Motor at 30 percent to home
                    hoodPID.setReference(Constants.HOMING_MOTOR__PERCENT_OUTPUT, CANSparkMax.ControlType.kDutyCycle);
                }

                // Executes this if homing has been going on for the MAX allotted time
                if (Timer.getFPGATimestamp() - homingStartTime > Constants.MAX_HOMING_TIME_S) {
                    // Sets motor speed to zero if max time is exceeded
                    hoodPID.setReference(0, CANSparkMax.ControlType.kDutyCycle);

                    // Sets current position as the 90 degree mark
                    hoodRelativeEncoder.setPosition(90);

                    DriverStation.reportWarning("Homing has taken longer than MAX expected time; homing has been stopped",
                            false);

                    // Turns homing off and sets it to the next queued state, OFF if no state queued
                    shooterState = nextState;

                    // Sends Homing start message to console
                    System.out.println("Homing Failed At: " + Timer.getFPGATimestamp());

                    // Resets Homing Start Time
                    homingStartTime = -1;
                }

                // Sets LED for Homing states
                setLedForHomingMode();
                break;

            case TEST:

                // Sets LED for testing state
                setLedForTestMode();

                break;
        }
    }

    private boolean isHoodStopped() {
        return Math.abs(hoodRelativeEncoder.getVelocity()) < Constants.HOOD_HAS_STOPPED_REFERENCE;
    }

    /**
     * Sets robot state to TEST mode.
     * <p>
     * Will activate self test that will help diagnose functionality of the Shooter Flywheel, Feeder Wheel, and Hood.
     */
    @Override
    public void selfTest() {
        shooterState = ShooterState.TEST;

        // Prints out start of test time
        System.out.println("Shooter Test Starting At: " + Timer.getFPGATimestamp());

        // Sets shooter to test speed
        setShooterSpeed(Constants.SHOOTER_TEST_SPEED_RPM);

        // Waits 5 seconds
        OrangeUtility.sleep(Constants.TEST_TIME_MS);

        // Turns shooter off
        setShooterSpeed(Constants.SHOOTER_TEST_SPEED_RPM);

        // Prints out start of test time
        System.out.println("Feeder Test Starting At: " + Timer.getFPGATimestamp());

        // Turns feeder on
        feederWheel.set(ControlMode.PercentOutput, 1);

        // Waits 5 seconds
        OrangeUtility.sleep(Constants.TEST_TIME_MS);

        // Turns feeder off
        feederWheel.set(ControlMode.PercentOutput, 0);

        // Prints out start of test time
        System.out.println("Hood Test Starting At: " + Timer.getFPGATimestamp());

        // Sets hood to 90 degrees
        setHoodPosition(90);

        // Waits 5 seconds
        OrangeUtility.sleep(Constants.TEST_TIME_MS);

        // Sets hood to 50 degrees
        setHoodPosition(50);

        // Waits 5 seconds
        OrangeUtility.sleep(Constants.TEST_TIME_MS);

        // Prints out start of test time
        System.out.println("All Tests Completed At: " + Timer.getFPGATimestamp());

        // Leaves TEST mode
        shooterState = ShooterState.OFF;
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
        logData("Shooter Flywheel Master Current", shooterWheelMaster.getSupplyCurrent());
        logData("Shooter Flywheel Slave Current", shooterWheelSlave.getSupplyCurrent());
        logData("Feeder Wheel Current", feederWheel.getSupplyCurrent());
        logData("Hood Motor Current", hoodMotor.getOutputCurrent());
    }

    /** Closing of Shooter motors is not supported. */
    @Override
    public void close() throws UnsupportedOperationException {
        throw new UnsupportedOperationException("Can not close this object");
    }
}
