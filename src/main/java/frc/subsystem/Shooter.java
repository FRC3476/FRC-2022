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

// TODO: Make javadocs
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
    double currentHomingPosition;

    // startTime and currentTime will be used to check how long the homing is occurring
    // Negative 1 used for a check to see if homingStartTime has been initialized in HOMING case
    double homingStartTime = -1;
    double homingCurrentTime;

    // Declarations of Modes and States
    public enum HoodPositionMode {ABSOLUTE_ENCODER, RELATIVE_TO_HOME}

    private HoodPositionMode hoodPositionMode;

    public enum HomeSwitchState {PRESSED, NOT_PRESSED}

    public enum FeederWheelState {ON, OFF}

    private FeederWheelState feederWheelState;

    public enum ShooterState {
        /**
         * Will turn off all motors
         */
        OFF,

        /**
         * Shooter is homing the hood
         */
        HOMING,

        /**
         * Flywheel is spinning and getting ready/is ready to shoot
         */
        ON,

        /**
         * Test state that will drive motors expected conditions
         */
        TEST
    }

    private ShooterState shooterState = ShooterState.ON;

    public enum NextState {
        /**
         * Will turn off all motors
         */
        OFF,

        /**
         * Shooter is homing the hood
         */
        HOMING,

        /**
         * Flywheel is spinning and getting ready/is ready to shoot
         */
        ON,

        /**
         * Test state that will drive motors expected conditions
         */
        TEST
    }

    private NextState nextState;

    // The target hood angle
    private double desiredHoodAngle;

    // Singleton Setup
    private static Shooter instance = new Shooter();

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
        feederWheel.config_kP(0, Constants.FEEDER_P, Constants.SHOOTER_PID_TIMEOUT_MS);
        feederWheel.config_kI(0, Constants.FEEDER_I, Constants.SHOOTER_PID_TIMEOUT_MS);
        feederWheel.config_kD(0, Constants.FEEDER_D, Constants.SHOOTER_PID_TIMEOUT_MS);
        feederWheel.config_kF(0, Constants.FEEDER_F, Constants.SHOOTER_PID_TIMEOUT_MS);
        feederWheel.config_IntegralZone(0, Constants.FEEDER_I_ZONE, Constants.SHOOTER_PID_TIMEOUT_MS);

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

    // Gets the hood angle for using the current encoder mode
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
        }

        // If using Relative Encoder
        else {
            hoodAngle = hoodRelativeEncoder.getPosition();
        }

        return hoodAngle;
    }

    private double getHoodAbsoluteEncoderValue() {
        return hoodAbsoluteEncoder.getOutput() * 360;
    }

    // Sets ShooterState to HOMING, HOMING will occur in update
    public void homeHood() {
        // Checks to make sure that robot is not currently HOMING or TESTING
        if (shooterState == ShooterState.HOMING || shooterState == ShooterState.TEST) {
            nextState = nextState.HOMING;
        } else {
            shooterState = ShooterState.HOMING;
        }
    }

    // Toggles between methods to get hood positions
    public void toggleHoodPositionMode() {
        // Checks current mode and switches to the alternate mode
        if (getHoodPositionMode() == HoodPositionMode.ABSOLUTE_ENCODER) {
            hoodPositionMode = HoodPositionMode.RELATIVE_TO_HOME;
            homeHood();
        } else {
            hoodPositionMode = HoodPositionMode.ABSOLUTE_ENCODER;
        }
    }

    // Returns if the Hood Position is being obtained through an absolute or relative encoder
    public HoodPositionMode getHoodPositionMode() {
        return hoodPositionMode;
    }

    // Sets Speed of Shooter Wheel
    public void setShooterSpeed(double desiredShooterSpeed) {
        this.desiredShooterSpeed = desiredShooterSpeed;
    }

    // Enables Feeder Wheel
    public void enableFeederWheel() {
        feederWheelState = FeederWheelState.ON;
    }

    // Disables Feeder Wheel
    public void disableFeederWheel() {
        feederWheelState = FeederWheelState.OFF;
    }

    // Gets state of feeder Wheel
    public FeederWheelState getFeederWheelState() {
        return feederWheelState;
    }

    // Updates the state of home switch and returns the state
    public HomeSwitchState getHomeSwitchState() {
        return homeSwitch.get() ? HomeSwitchState.PRESSED : HomeSwitchState.NOT_PRESSED;
    }

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

    public double getShooterRPM() {
        // Convert Falcon 500 encoder units for velocity into RPM
        return shooterWheelMaster.getSelectedSensorVelocity() * Constants.FALCON_UNIT_CONVERSION_FOR_RELATIVE_ENCODER;
    }

    // Returns desired/commanded shooter speed
    public double getDesiredShooterSpeed() {
        return desiredShooterSpeed;
    }

    // Checks if shooter is at target speed with a configurable allowed error
    public boolean isShooterAtTargetSpeed() {
        return getShooterRPM() > Math.abs(getDesiredShooterSpeed() - (Constants.ALLOWED_SHOOTER_SPEED_ERROR / 2.0)) &&
                getShooterRPM() < Math.abs(getDesiredShooterSpeed() + (Constants.ALLOWED_SHOOTER_SPEED_ERROR / 2.0));
    }

    // Sets shooter state to OFF, will lock motors
    public void turnShooterOFF() {
        // Checks to make sure that HOMING or TEST is not occurring
        if (shooterState == ShooterState.HOMING || shooterState == ShooterState.TEST) {
            nextState = nextState.OFF;
        } else {
            shooterState = ShooterState.HOMING.OFF;
        }
    }

    // Turns shooter ON
    public void turnShooterON() {
        // Checks to make sure that HOMING or TEST is not occurring
        if (shooterState == ShooterState.HOMING || shooterState == ShooterState.TEST) {
            nextState = NextState.ON;
        } else {
            shooterState = ShooterState.ON;
        }
    }

    // Checks if Hood is at the target angle with a configurable allowed error
    public boolean isHoodAtTargetAngle() {
        return getHoodAngle() > (Math.abs(getDesiredHoodAngle() - Constants.ALLOWED_HOOD_ANGLE_ERROR) / 2.0) &&
                getHoodAngle() < (Math.abs(getDesiredHoodAngle() + Constants.ALLOWED_HOOD_ANGLE_ERROR) / 2.0);
    }

    // Returns desired/commanded hood angle
    public double getDesiredHoodAngle() {
        return desiredHoodAngle;
    }

    // Returns current Shooter State, can be OFF, ON, HOMING, or TEST
    public ShooterState getShooterState() {
        return shooterState;
    }

    /**
     * Returns queued state for states that have been requested but could not be changed immediately
     *
     * @return returns queued state, may be null if there is no queued state
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
        }

        // Defaults to ON if next state is not specified
        else {
            shooterState = shooterState.ON;
        }

        // Resets nextState
        nextState = null;
    }

    // TODO: Fill out the rest of the LED methods
    // Sets Blinkin LED to color when robot is OFF
    private void setLedForOffMode() {
        BlinkinLED.getInstance().setColor(Constants.LED_SHOOTER_OFF);
    }

    // Sets LED different colors, depends on if flywheel is up to speed and if hood is in correct position
    private void setLedForOnMode() {

        // Sets LED different colors for different shooter scenarios
        // If Shooter is not at the target speed, LED will display this color
        if (isShooterAtTargetSpeed() == false) {
            BlinkinLED.getInstance().setColor(Constants.LED_FLYWHEEL_APPROACHING_DESIRED_SPEED);
        }

        // If Shooter is not at the target angle, but it is at the target speed, LED will display this color
        else if (isHoodAtTargetAngle() == false) {
            BlinkinLED.getInstance().setColor(Constants.LED_HOOD_APPROACHING_DESIRED_POSITION);
        }

        // If both hood and flywheel are ready for shooting, LED will display this color
        else {
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
                }

                // Executes this if home switch is not pressed
                if (getHomeSwitchState() == HomeSwitchState.NOT_PRESSED) {
                    // Gets current time
                    homingCurrentTime = Timer.getFPGATimestamp();

                    // Gets current encoder value
                    currentHomingPosition = hoodRelativeEncoder.getPosition();

                    // Only executes if last homing increment has finished
                    if (Math.abs(hoodRelativeEncoder.getVelocity()) < 1.0e-3) {
                        hoodPID.setReference(currentHomingPosition + Constants.HOMING_PRECISION_IN_MOTOR_ROTATIONS,
                                CANSparkMax.ControlType.kPosition);
                    }
                }
                // If home switch is pressed
                else {
                    // Sets the relative encoder reference to the position of the home switch when Home switch is pressed
                    hoodRelativeEncoder.setPosition(90);

                    // Turns homing off and sets it to the next queued state, ON if no state queued
                    setNextStateToCurrentState();
                }

                // Executes this if homing has been going on for the MAX allotted time
                if (homingCurrentTime - homingStartTime > Constants.MAX_HOMING_TIME_S) {
                    DriverStation.reportWarning("Homing has taken longer than MAX expected time; homing has been stopped",
                            false);

                    // Turns homing off and sets it to the next queued state, ON if no state queued
                    setNextStateToCurrentState();
                }

                // Sets LED for Homing states
                setLedForHomingMode();
                break;

            case TEST:

                // Turns TESTING off and sets it to the next queued state, ON if no state queued
                setNextStateToCurrentState();

                // Sets LED for testing states
                setLedForTestMode();
                break;
        }
    }

    @Override
    public void selfTest() {
        // Checks to make sure that robot is not currently HOMING or TESTING
        if (shooterState == ShooterState.HOMING || shooterState == ShooterState.TEST) {
            nextState = nextState.TEST;
        } else {
            shooterState = ShooterState.TEST;
        }
    }

    // Logs variety of values and states to ShuffleBoard
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
    }

    @Override
    public void close() throws UnsupportedOperationException {
        throw new UnsupportedOperationException("Can not close this object");
    }
}
