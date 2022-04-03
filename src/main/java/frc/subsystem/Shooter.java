package frc.subsystem;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.utility.OrangeUtility;
import frc.utility.Timer;
import frc.utility.controllers.LazyCANSparkMax;
import frc.utility.controllers.LazyTalonFX;
import frc.utility.shooter.visionlookup.ShooterPreset;
import org.jetbrains.annotations.NotNull;

import static frc.robot.Constants.*;

/**
 * Shooter class controls the shooter flywheel, feeder wheel, and variable hood Has motor control wrappers for setting velocity
 * and position for respective parts.
 * <p>
 * Can get speeds of motor and states / modes included in the class Has modes for HOMING and TESTING.
 * <p>
 * Has support for logging to Shuffleboard Communicates some shooting, testing, and homing states with BlinkinLED.
 */

public final class Shooter extends AbstractSubsystem {

    final @NotNull NetworkTableEntry shooterP = SmartDashboard.getEntry("ShooterPIDP");
    final @NotNull NetworkTableEntry shooterI = SmartDashboard.getEntry("ShooterPIDI");
    final @NotNull NetworkTableEntry shooterD = SmartDashboard.getEntry("ShooterPIDD");
    final @NotNull NetworkTableEntry shooterF = SmartDashboard.getEntry("ShooterPIDF");
    final @NotNull NetworkTableEntry shooterIZone = SmartDashboard.getEntry("ShooterPIDIZone");

    // Talon500 Initialization

    // Shooter Flywheel
    private final LazyTalonFX shooterWheelMaster;
    private final LazyTalonFX shooterWheelSlave;
    private double desiredShooterSpeed;

    // Feeder
    private final LazyTalonFX feederWheel;

    // Hood
    private final LazyCANSparkMax hoodMotor;
    private SparkMaxPIDController hoodPID;
    private final RelativeEncoder hoodRelativeEncoder;

    private final CANCoder hoodAbsoluteEncoder;

    // Home Switch Initialization
    private final DigitalInput homeSwitch;

    // Homing initialization

    // startTime and currentTime will be used to check how long the homing is occurring
    // Negative 1 used for a check to see if homingStartTime has been initialized in HOMING case
    private double homingStartTime = -1;

    // Makes it so feeder can run when not at proper hood and or flywheel speed
    private boolean feederChecksDisabled = false;

    public volatile boolean runFeederWheelReversed;

    /**
     * @return true when the shooter will be firing.
     */
    public boolean isFiring() {
        return shooterState == ShooterState.ON &&
                ((feederWheelState == FeederWheelState.FORWARD)
                        && ((isHoodAtTargetAngle() && isShooterAtTargetSpeed()) || feederChecksDisabled));
    }

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

        REVERSE
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
        // Sets update method's iteration
        super(Constants.SHOOTER_PERIOD_MS, 2);

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

        feederWheel.setNeutralMode(NeutralMode.Brake);

        configPID();

        hoodRelativeEncoder.setPosition(hoodAbsoluteEncoder.getPosition());

        shooterP.setDouble(Constants.DEFAULT_SHOOTER_P);
        shooterI.setDouble(Constants.DEFAULT_SHOOTER_I);
        shooterD.setDouble(Constants.DEFAULT_SHOOTER_D);
        shooterF.setDouble(Constants.DEFAULT_SHOOTER_F);
        shooterIZone.setDouble(Constants.DEFAULT_SHOOTER_IZONE);

        shooterP.addListener(event -> shooterWheelMaster.config_kP(0, event.getEntry().getDouble(Constants.DEFAULT_SHOOTER_P)),
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        shooterI.addListener(event -> shooterWheelMaster.config_kI(0, event.getEntry().getDouble(Constants.DEFAULT_SHOOTER_I)),
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        shooterD.addListener(event -> shooterWheelMaster.config_kD(0, event.getEntry().getDouble(Constants.DEFAULT_SHOOTER_D)),
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        shooterF.addListener(event -> shooterWheelMaster.config_kF(0, event.getEntry().getDouble(Constants.DEFAULT_SHOOTER_F)),
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        shooterIZone.addListener(
                event -> shooterWheelMaster.config_IntegralZone(0, event.getEntry().getDouble(Constants.DEFAULT_SHOOTER_IZONE)),
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }

    private void configPID() {
        // Second Falcon Follows Speed Of Main Falcon
        shooterWheelSlave.follow(shooterWheelMaster);

        // Configure PID Constants and current limit
        shooterWheelMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        shooterWheelMaster.configPeakOutputForward(1);
        shooterWheelMaster.configPeakOutputReverse(0);
        shooterWheelMaster.configVoltageCompSaturation(9);
        shooterWheelSlave.configVoltageCompSaturation(9);

        shooterWheelMaster.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.SHOOTER_CURRENT_LIMIT,
                Constants.SHOOTER_TRIGGER_THRESHOLD_CURRENT, Constants.SHOOTER_TRIGGER_THRESHOLD_TIME));
        shooterWheelSlave.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.SHOOTER_CURRENT_LIMIT,
                Constants.SHOOTER_TRIGGER_THRESHOLD_CURRENT, Constants.SHOOTER_TRIGGER_THRESHOLD_TIME));

        shooterWheelMaster.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);

        shooterWheelMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20); // Default is 10ms
        shooterWheelMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 25); // Default is 10ms
        shooterWheelMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 100); // Default is 50ms
        shooterWheelMaster.setControlFramePeriod(ControlFrame.Control_3_General, 25);
        shooterWheelMaster.setControlFramePeriod(ControlFrame.Control_4_Advanced, 25);
        shooterWheelMaster.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 500);

        shooterWheelSlave.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 100); // Default is 10ms
        shooterWheelSlave.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 100); // Default is 10ms
        shooterWheelSlave.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 100); // Default is 50ms
        shooterWheelSlave.setControlFramePeriod(ControlFrame.Control_3_General, 25);
        shooterWheelSlave.setControlFramePeriod(ControlFrame.Control_4_Advanced, 25);
        shooterWheelSlave.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 500);

        // Turns off brake mode for shooter flywheel
        shooterWheelMaster.setNeutralMode(NeutralMode.Coast);
        shooterWheelSlave.setNeutralMode(NeutralMode.Coast);


        // This PID setup for 775pro will not be used
        feederWheel.config_kP(0, Constants.FEEDER_WHEEL_P);
        feederWheel.config_kI(0, Constants.FEEDER_WHEEL_I);
        feederWheel.config_kD(0, Constants.FEEDER_WHEEL_D);
        feederWheel.config_kF(0, Constants.FEEDER_WHEEL_F);
        feederWheel.config_IntegralZone(0, Constants.FEEDER_WHEEL_I_ZONE);
        feederWheel.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.FEEDER_CURRENT_LIMIT,
                Constants.FEEDER_TRIGGER_THRESHOLD_CURRENT, Constants.FEEDER_TRIGGER_THRESHOLD_TIME));

        feederWheel.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 200); // Default is 10ms
        feederWheel.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 200); // Default is 10ms
        feederWheel.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 100); // Default is 50ms
        feederWheel.setControlFramePeriod(ControlFrame.Control_3_General, 25);
        feederWheel.setControlFramePeriod(ControlFrame.Control_4_Advanced, 25);
        feederWheel.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 500);
        feederWheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        feederWheel.configPeakOutputReverse(-1);
        feederWheel.configPeakOutputForward(1);

        hoodPID = hoodMotor.getPIDController();
        hoodPID.setP(Constants.HOOD_P, 0);
        hoodPID.setI(Constants.HOOD_I, 0);
        hoodPID.setD(Constants.HOOD_D, 0);
        hoodPID.setFF(Constants.HOOD_F, 0);
        hoodPID.setIZone(Constants.HOOD_I_ZONE);
        hoodPID.setOutputRange(-Constants.HOOD_MAX_OUTPUT, Constants.HOOD_MAX_OUTPUT);
        hoodMotor.setSmartCurrentLimit(Constants.HOOD_CURRENT_LIMIT_AMPS);
        hoodMotor.setInverted(true);
        hoodMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        hoodMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
        hoodMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 25);
        hoodMotor.setControlFramePeriodMs(25);

        hoodAbsoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 200);
        hoodAbsoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 25);
        hoodAbsoluteEncoder.configSensorDirection(true);

        hoodMotor.burnFlash();
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
            hoodAngle = getHoodAbsoluteEncoderValue();

            // Checks if Absolute Encoder is reading outside expected range
//            if (hoodAngle > 90.5 || hoodAngle < 49.5) {
//                DriverStation.reportWarning("Hood position reading values over MAX expected.\n" +
//                        "Should switch to home relative", false);
//            }
        } else {
            // If using Relative Encoder
            hoodAngle = hoodRelativeEncoder.getPosition();
        }

        return hoodAngle;
    }

    // Raw degree measurement from Absolute Encoder, The angle may need an offset
    private double getHoodAbsoluteEncoderValue() {
        return hoodAbsoluteEncoder.getAbsolutePosition();
    }

    private double getHoodRelativeAngle() {
        return hoodRelativeEncoder.getPosition();
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
     * ABSOLUTE_ENCODER uses the absolute encoder to correct the built-in encoder in the neo.
     * <p>
     * RELATIVE_TO_HOME only uses the built-in encoder in the neo.
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


    public void set(ShooterPreset config) {
        setSpeed(config.getFlywheelSpeed());
        setHoodPosition(config.getHoodEjectAngle());
    }

    /**
     * Sets Speed of Shooter FlywheelWheel.
     *
     * @param desiredShooterSpeed Desired Speed in RPM.
     */
    public void setSpeed(double desiredShooterSpeed) {
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

    public void reverseShooterWheel() {
        feederWheelState = FeederWheelState.REVERSE;
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
        if (desiredAngle < Constants.HOOD_MIN_ANGLE) {
            desiredAngle = Constants.HOOD_MIN_ANGLE;
        }

        if (desiredAngle > Constants.HOOD_MAX_ANGLE) {
            desiredAngle = Constants.HOOD_MAX_ANGLE;
        }
        desiredHoodAngle = desiredAngle;
    }

    /**
     * Gets the angular speed of the shooter flywheel in RPM.
     */
    public double getShooterRPM() {
        // Convert Falcon 500 encoder units for velocity into RPM
        return shooterWheelMaster.getSelectedSensorVelocity() / Constants.SET_SHOOTER_SPEED_CONVERSION_FACTOR;
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

    private void moveHoodMotor() {
        hoodPID.setReference((desiredHoodAngle - getHoodAngle() + hoodRelativeEncoder.getPosition()),
                CANSparkMax.ControlType.kPosition);
    }

    public void setFeederChecksDisabled(boolean feederChecksDisabled) {
        this.feederChecksDisabled = feederChecksDisabled;
    }

    public boolean isFeederChecksDisabled() {
        return feederChecksDisabled;
    }

    public void forceFeederForward() {
        feederWheel.set(ControlMode.PercentOutput, Constants.FEEDER_WHEEL_SPEED);
    }

    public void setHoodZero() {
        System.out.println(" Setting Zero " + hoodAbsoluteEncoder.configGetMagnetOffset() + " -> 90");
        hoodAbsoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        hoodAbsoluteEncoder.configMagnetOffset(
                -(-hoodAbsoluteEncoder.getAbsolutePosition() - hoodAbsoluteEncoder.configGetMagnetOffset()) - 90);
    }

    public double getLastShotTime() {
        return lastShotTime;
    }

    private volatile double lastShotTime = 0;

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
                Robot.setRumble(RumbleType.kLeftRumble, 0);
                shooterWheelMaster.set(ControlMode.PercentOutput, 0);
                setHoodPosition(Constants.HOOD_MAX_ANGLE); // Sets hood to the lowest possible position

                if (!isHoodAtTargetAngle()) {
                    moveHoodMotor();
                }

                if (feederWheelState == FeederWheelState.REVERSE) {
                    feederWheel.set(ControlMode.PercentOutput, -Constants.FEEDER_WHEEL_SPEED);
                } else {
                    if (runFeederWheelReversed) {
                        feederWheel.set(ControlMode.PercentOutput, -0.3);
                    } else {
                        feederWheel.set(ControlMode.PercentOutput, 0);
                    }
                }

                Robot.setRumble(RumbleType.kLeftRumble, 0);

                break;

            case ON:

                shooterWheelMaster.set(ControlMode.Velocity,
                        desiredShooterSpeed * Constants.SET_SHOOTER_SPEED_CONVERSION_FACTOR); // Sets shooter motor to desired shooter

                if (!isHoodAtTargetAngle()) {
                    moveHoodMotor(); // Sets Motor to travel to desired hood angle
                }

                if (lastShotTime > Timer.getFPGATimestamp()) {
                    //Check to make sure we don't accidentally do anything dumb and prevent us from shooting. It checks if the
                    //nextAllowedShootTime is greater than the current time plus the delay time which should never happen
                    lastShotTime = Timer.getFPGATimestamp();
                }

                if (
                        ((feederWheelState == FeederWheelState.FORWARD)
                                && ((isHoodAtTargetAngle() && isShooterAtTargetSpeed())
                                && (Timer.getFPGATimestamp() > (VisionManager.getInstance().getDistanceToTarget() < 80 ?
                                0.5 : SECOND_BALL_SHOOT_DELAY) + lastShotTime)
                                || feederChecksDisabled))
                ) {
                    feederWheel.set(ControlMode.PercentOutput, FEEDER_WHEEL_SPEED);
                    lastShotTime = Timer.getFPGATimestamp();
                    Robot.setRumble(RumbleType.kLeftRumble, 0);
                } else {
                    // Turn OFF Feeder Wheel if feederWheel has not been on in half a second
                    if (Timer.getFPGATimestamp() > lastShotTime + FEEDER_CHANGE_STATE_DELAY_SEC) {
                        if (runFeederWheelReversed) {
                            feederWheel.set(ControlMode.PercentOutput, -0.3);
                        } else {
                            feederWheel.set(ControlMode.PercentOutput, 0);
                        }
                    }

                    if (Timer.getFPGATimestamp() > lastShotTime + RUMBLE_DELAY
                            && Timer.getFPGATimestamp() < lastShotTime + RUMBLE_DELAY + RUMBLE_TIME) {
                        Robot.setRumble(RumbleType.kLeftRumble, 0.25);
                    } else {
                        Robot.setRumble(RumbleType.kLeftRumble, 0);
                    }
                }
                break;

            case HOMING:
                // Does homing sequence if state is set to HOMING
                // Will turn motor towards home switch until home switch is enabled

                // Checks if homing has started yet
                //noinspection FloatingPointEquality
                if (homingStartTime == -1) {
                    // Records start of homing time, with current time being the same as the start time
                    if (DriverStation.isEnabled()) {
                        homingStartTime = Timer.getFPGATimestamp();
                        System.out.println("Homing Starting At: " + homingStartTime);
                    } else {
                        return;
                    }
                }

                // Executes this if home switch is pressed
                if (getHomeSwitchState() == HomeSwitchState.PRESSED || hoodMotor.getOutputCurrent() > Constants.SHOOTER_HOMING_CURRENT_LIMIT) {
                    hoodPID.setReference(0, CANSparkMax.ControlType.kDutyCycle); // Stop the hood motor
                    // Sets the relative encoder reference to the position of the home switch when Home switch is pressed
                    hoodRelativeEncoder.setPosition(Constants.HOOD_MAX_ANGLE);


                    // Turns homing off and sets it to the next queued state, ON if no state queued
                    shooterState = nextState;

                    // Sends Homing start message to console
                    System.out.println("Homing Finished Successfully At: " + Timer.getFPGATimestamp() +
                            (hoodMotor.getOutputCurrent() > 10 ? " (Hood Motor Current Exceeded)" : ""));

                    // Resets Homing Start Time
                    homingStartTime = -1;
                } else {

                    // Runs Motor at 30 percent to home
                    hoodPID.setReference(Constants.HOMING_MOTOR_PERCENT_OUTPUT, CANSparkMax.ControlType.kDutyCycle);
                }

                // Executes this if homing has been going on for the MAX allotted time
                if (Timer.getFPGATimestamp() - homingStartTime > Constants.MAX_HOMING_TIME_S) {
                    // Sets motor speed to zero if max time is exceeded
                    hoodPID.setReference(0, CANSparkMax.ControlType.kDutyCycle);
                    // Sets current position as the 90 degree mark
                    hoodRelativeEncoder.setPosition(Constants.HOOD_MAX_ANGLE);

                    DriverStation.reportWarning("Homing has taken longer than MAX expected time; homing has been stopped",
                            false);
                    shooterState = nextState;
                    System.out.println("Homing Failed At: " + Timer.getFPGATimestamp());
                    homingStartTime = -1;
                }
                Robot.setRumble(RumbleType.kLeftRumble, 0);
                break;

            case TEST:
                Robot.setRumble(RumbleType.kLeftRumble, 0);
                break;
        }
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
        setSpeed(Constants.SHOOTER_TEST_SPEED_RPM);

        // Waits 5 seconds
        OrangeUtility.sleep(Constants.TEST_TIME_MS);

        // Turns shooter off
        setSpeed(Constants.SHOOTER_TEST_SPEED_RPM);

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
        setHoodPosition(Constants.HOOD_MAX_ANGLE);

        // Waits 5 seconds
        OrangeUtility.sleep(Constants.TEST_TIME_MS);

        // Sets hood to 50 degrees
        setHoodPosition(Constants.HOOD_MIN_ANGLE);

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
        logData("Shooter Native Flywheel SPeed", getDesiredShooterSpeed() * Constants.SET_SHOOTER_SPEED_CONVERSION_FACTOR);
        logData("Hood Angle", getHoodAngle());
        logData("Relative Hood Angle", getHoodRelativeAngle());
        logData("Desired Shooter Speed", getDesiredShooterSpeed());
        logData("Desired Hood Angle", getDesiredHoodAngle());
        logData("Feeder Wheel State", getFeederWheelState());
        logData("Feeder Wheel Speed", feederWheel.getSelectedSensorVelocity() * Constants.FALCON_ENCODER_TICKS_PER_100_MS_TO_RPM);
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
