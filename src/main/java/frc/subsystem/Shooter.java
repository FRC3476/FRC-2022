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


// TODO: Need to add Blinkin LED for the current states of things
public class Shooter extends AbstractSubsystem {

    // Talon500 Initialization
    private LazyTalonFX shooterWheelMaster;
    private LazyTalonFX shooterWheelSlave;
    private double desiredShooterSpeed;

    // 775Pro Initialization
    private LazyTalonSRX feederWheel;

    // NEO550 Initialization
    private LazyCANSparkMax hoodMotor;
    private SparkMaxPIDController hoodPID;
    private RelativeEncoder hoodRelativeEncoder;

    // REV Through Bore Encoder Initialization
    private DutyCycle hoodAbsoluteEncoder;

    // Home Switch Initialization
    private DigitalInput homeSwitch;

    // Declarations of Modes and States
    private enum HoodPositionModes {ABSOLUTE_ENCODER, RELATIVE_TO_HOME}

    private HoodPositionModes hoodPositionMode;

    private enum HomeSwitchStates {PRESSED, NOT_PRESSED}

    private HomeSwitchStates homeSwitchState;

    private enum FeederWheelStates {ON, OFF}

    private FeederWheelStates feederWheelState;

    // The target hood angle
    private double desiredHoodAngle;

    // Singleton Setup
    private static final Shooter instance = new Shooter();

    public static Shooter getInstance() {
        return instance;
    }

    // Private constructor for singleton
    private Shooter() {
        // TODO: May have to invert direction of motors
        // Sets update method's iteration
        super(Constants.SHOOTER_PERIOD_MS);

        // Sets hood position mode, can be either using absolute encoder or be relative to home switch
        hoodPositionMode = HoodPositionModes.ABSOLUTE_ENCODER;

        // Sets roboRIO IDs to each component
        shooterWheelMaster = new LazyTalonFX(Constants.SHOOTER_WHEEL_CAN_MASTER_ID);
        shooterWheelSlave = new LazyTalonFX(Constants.SHOOTER_WHEEL_CAN_SLAVE_ID);
        feederWheel = new LazyTalonSRX(Constants.FEEDER_WHEEL_CAN_ID);
        hoodMotor = new LazyCANSparkMax(Constants.HOOD_MOTOR_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        hoodAbsoluteEncoder = new DutyCycle(new DigitalInput(Constants.HOOD_ENCODER_DIO_ID));
        hoodRelativeEncoder = hoodMotor.getEncoder();
        hoodRelativeEncoder.setPositionConversionFactor(Constants.HOOD_DEGREES_PER_MOTOR_ROTATION);
        homeSwitch = new DigitalInput(Constants.HOOD_HOME_SWITCH_DIO_ID);

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
        double hoodAngle = 0;

        // If using Absolute Encoder
        if (hoodPositionMode == HoodPositionModes.ABSOLUTE_ENCODER) {
            // TODO: Figure out where encoder will be placed and convert to correct angle if necessary
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

    // Find Home switch
    // Will turn motor towards home switch until home switch is enabled
    public void homeHood() {

        double currentPosition = 0;

        double startTime = Timer.getFPGATimestamp();
        double currentTime = startTime;

        while (getHomeSwitchState() == HomeSwitchStates.PRESSED && currentTime - startTime > Constants.MAX_HOMING_TIME_S) {
            // Gets current time
            currentTime = Timer.getFPGATimestamp();
            // Gets encoder value at start of homing
            currentPosition = hoodRelativeEncoder.getPosition();

            // Will only set to next position when motor has completed last increment
            if (Math.abs(hoodMotor.getEncoder().getVelocity()) < 1.0e-3) {
                hoodPID.setReference(currentPosition + Constants.HOMING_PRECISION_IN_MOTOR_ROTATIONS,
                        CANSparkMax.ControlType.kPosition);
            }
        }
        hoodRelativeEncoder.setPosition(90);
    }

    // Toggles between methods to get hood positions
    public void toggleHoodPositionMode() {
        // Checks current mode and switches to the alternate mode
        if (getHoodPositionMode() == HoodPositionModes.ABSOLUTE_ENCODER) {
            hoodPositionMode = HoodPositionModes.RELATIVE_TO_HOME;
            homeHood();
        } else {
            hoodPositionMode = HoodPositionModes.ABSOLUTE_ENCODER;
        }
    }

    // Returns if the Hood Position is being obtained through an absolute or relative encoder
    public HoodPositionModes getHoodPositionMode() {
        return hoodPositionMode;
    }

    // Sets Speed of Shooter Wheel
    // TODO: Figure out of input speed needs to be converted to work with gearbox
    public void setShooterSpeed(double desiredShooterSpeed) {
        this.desiredShooterSpeed = desiredShooterSpeed;
    }

    // Enables Feeder Wheel
    public void enableFeederWheel() {
        feederWheelState = FeederWheelStates.ON;
    }

    // Disables Feeder Wheel
    public void disableFeederWheel() {
        feederWheelState = FeederWheelStates.OFF;
    }

    // Gets state of feeder Wheel
    public FeederWheelStates getFeederWheelState() {
        return feederWheelState;
    }

    // Updates the state of home switch and returns the state
    public HomeSwitchStates getHomeSwitchState() {
        if (homeSwitch.get()) {
            homeSwitchState = HomeSwitchStates.PRESSED;
        } else {
            homeSwitchState = HomeSwitchStates.NOT_PRESSED;
        }
        return homeSwitchState;
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

    // TODO: Make sure this is accurate as motor is going into gearbox
    public double getShooterRPM() {
        // Convert Falcon 500 encoder units for velocity into RPM
        return shooterWheelMaster.getSelectedSensorVelocity() * Constants.FALCON_UNIT_CONVERSION_FOR_RELATIVE_ENCODER;
    }

    public double getDesiredShooterSpeed() {
        return desiredShooterSpeed;
    }

    public boolean isShooterAtTargetSpeed() {
        return getShooterRPM() > Math.abs(getDesiredShooterSpeed() - (Constants.ALLOWED_SHOOTER_SPEED_ERROR / 2.0)) &&
                getShooterRPM() < Math.abs(getDesiredShooterSpeed() + (Constants.ALLOWED_SHOOTER_SPEED_ERROR / 2.0));
    }

    @Override
    public void update() {
        // Sets shooter motor to desired shooter speed
        shooterWheelMaster.set(ControlMode.Velocity, desiredShooterSpeed);

        // Updates the state of the home switch to PRESSED or NOT_PRESSED
        if (homeSwitch.get()) {
            homeSwitchState = HomeSwitchStates.PRESSED;
        } else {
            homeSwitchState = HomeSwitchStates.NOT_PRESSED;
        }

        // Sets Motor to travel to desired hood angle
        hoodPID.setReference((desiredHoodAngle - getHoodAngle()), CANSparkMax.ControlType.kPosition);

        // Checks to see if feeder wheel is enabled, if hoodMotor had finished moving, and if shooterWheel is at target speed
        if ((feederWheelState == FeederWheelStates.ON) && Math.abs(hoodMotor.getEncoder().getVelocity()) < 1.0e-3 &&
                isShooterAtTargetSpeed()) {

            // Set Feeder wheel to MAX speed
            feederWheel.set(ControlMode.PercentOutput, 1);
        } else {
            // Turn OFF Feeder Wheel
            feederWheel.set(ControlMode.PercentOutput, 0);
        }
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
