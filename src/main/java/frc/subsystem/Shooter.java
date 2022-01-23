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

    // Talon500
    private LazyTalonFX shooterWheelMaster;
    private LazyTalonFX shooterWheelSlave;

    // 775Pro
    private LazyTalonSRX feederWheel;
    private boolean feederWheelState;

    // NEO550
    private LazyCANSparkMax hoodMotor;
    private SparkMaxPIDController hoodPID;
    private RelativeEncoder hoodRelativeEncoder;

    // REV Through Bore Encoder
    private DutyCycle hoodAbsoluteEncoder;

    // Home Switch
    private DigitalInput homeSwitch;

    private enum HoodPositionModes {ABSOLUTE_ENCODER, RELATIVE_TO_HOME}

    private HoodPositionModes hoodPositionMode;

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


    // TODO: May have to add an offset for the encoder if 0 degrees is not on the 90 degree from horizontal
    // angle should be between 0 and 40 representing the 90 and 50 degree of the horizontal
    private double getHoodPosition() {

        double angle = 0;
        if (hoodPositionMode == HoodPositionModes.ABSOLUTE_ENCODER) {
            // TODO: Figure out where encoder will be placed and convert to correct angle if necessary
            angle = getHoodAbsoluteEncoderValue() + Constants.HOOD_ABSOLUTE_ENCODER_OFFSET; // Put in a wrapper method

            // Checks if Absolute Encoder is reading outside of expected range
            if (angle > 90.5 || angle < 49.5) {
                DriverStation.reportWarning("Hood position reading values over MAX expected.\n" +
                        "Should switch to home relative", false);
            }
        }
        else
        {
            angle = hoodRelativeEncoder.getPosition();
        }
        return angle;
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


        while (homeSwitch.get() == false && currentTime - startTime > Constants.MAX_HOMING_TIME_S) {
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
        // Checks current Hood position mode to tell which to switch to
        if (getHoodPositionMode() == HoodPositionModes.ABSOLUTE_ENCODER) {
            hoodPositionMode = HoodPositionModes.RELATIVE_TO_HOME;
            homeHood();
        } else {
            hoodPositionMode = HoodPositionModes.ABSOLUTE_ENCODER;
        }
    }

    public HoodPositionModes getHoodPositionMode() {
        return hoodPositionMode;
    }

    private void setShooterSpeed(double speed) {
        shooterWheelMaster.set(ControlMode.Velocity, speed);
    }

    // TODO: Feeder wheel needs to check if hood is at proper position and wheel is at desired speed
    private void setFeederWheelState(boolean state) {
        feederWheelState = state;
    }

    private void setHoodPosition(double desiredAngle) {
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

    @Override
    public void update() {
        // Set hood to desired position
        hoodPID.setReference((desiredHoodAngle - getHoodPosition()), CANSparkMax.ControlType.kPosition);

        // Checks to see if feeder wheel is enabled, if hoodMotor had finished moving, and if shooterWheel is at target speed
        if ((feederWheelState == true) && Math.abs(hoodMotor.getEncoder().getVelocity()) < 1.0e-3 &&) {
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
