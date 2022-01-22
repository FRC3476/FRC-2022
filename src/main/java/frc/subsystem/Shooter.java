package frc.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
import frc.robot.Constants;
import frc.utility.controllers.LazyCANSparkMax;
import frc.utility.controllers.LazyTalonSRX;



public class Shooter extends AbstractSubsystem {

    // Talon500
    private LazyTalonSRX shooterWheelMaster;
    private LazyTalonSRX shooterWheelSlave;

    // 775Pro
    private LazyTalonSRX feederWheel;

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
        shooterWheelMaster = new LazyTalonSRX(Constants.SHOOTER_WHEEL_CAN_MASTER_ID);
        shooterWheelSlave = new LazyTalonSRX(Constants.SHOOTER_WHEEL_CAN_SLAVE_ID);
        feederWheel = new LazyTalonSRX(Constants.FEEDER_WHEEL_CAN_ID);
        hoodMotor = new LazyCANSparkMax(Constants.HOOD_MOTOR_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        hoodAbsoluteEncoder = new DutyCycle(new DigitalInput(Constants.HOOD_ENCODER_DIO_ID));
        hoodRelativeEncoder = hoodMotor.getEncoder();
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
        shooterWheelMaster.enableCurrentLimit(true);
        shooterWheelMaster.configContinuousCurrentLimit(Constants.SHOOTER_CURRENT_LIMIT_AMPS);


        // This PID setup for 775pro will not be used
        feederWheel.config_kP(0, Constants.FEEDER_P, Constants.SHOOTER_PID_TIMEOUT_MS);
        feederWheel.config_kI(0, Constants.FEEDER_I, Constants.SHOOTER_PID_TIMEOUT_MS);
        feederWheel.config_kD(0, Constants.FEEDER_D, Constants.SHOOTER_PID_TIMEOUT_MS);
        feederWheel.config_kF(0, Constants.FEEDER_F, Constants.SHOOTER_PID_TIMEOUT_MS);
        feederWheel.config_IntegralZone(0, Constants.FEEDER_I_ZONE, Constants.SHOOTER_PID_TIMEOUT_MS);
        feederWheel.enableCurrentLimit(true);
        feederWheel.configContinuousCurrentLimit(Constants.FEEDER_CURRENT_LIMIT_AMPS);

        hoodPID = hoodMotor.getPIDController();
        hoodPID.setP(Constants.HOOD_P, 0);
        hoodPID.setI(Constants.HOOD_I,0);
        hoodPID.setD(Constants.HOOD_D,0);
        hoodPID.setFF(Constants.HOOD_F,0);
        hoodPID.setIZone(Constants.HOOD_I_ZONE);
        hoodMotor.setSmartCurrentLimit(Constants.HOOD_CURRENT_LIMIT_AMPS);
    }


    // TODO: May have to add an offset for the encoder if 0 degrees is not on the 90 degree from horizontal
    // angle should be between 0 and 40 representing the 90 and 50 degree of the horizontal
    private double getHoodPosition() {

        double angle = 0;
        if (hoodPositionMode == HoodPositionModes.ABSOLUTE_ENCODER) {
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
    private void homeHood() {
        // Gets encoder value at start of homing
        double currentPosition = hoodRelativeEncoder.getPosition();
        // Will turn motor towards home switch until home switch is enabled
        while (homeSwitch.get()) {
            // Will only set to next position when motor has completed last increment
            if (Math.abs(hoodMotor.getEncoder().getVelocity()) > 1.0e-3) {
                hoodPID.setReference(currentPosition + .1, CANSparkMax.ControlType.kPosition);
            }
        }
        hoodRelativeEncoder.setPosition(90);
    }

    // Toggles between methods to get hood positions
    private void toggleHoodPositionMode() {
        // Checks current Hood position mode to tell which to switch to
        if (getHoodPositionMode() == HoodPositionModes.ABSOLUTE_ENCODER) {
            hoodPositionMode = HoodPositionModes.RELATIVE_TO_HOME;
        } else {
            hoodPositionMode = HoodPositionModes.ABSOLUTE_ENCODER;
        }
    }

    private HoodPositionModes getHoodPositionMode() {
        return hoodPositionMode;
    }

    private void setShooterSpeed(double speed) {
        shooterWheelMaster.set(ControlMode.Velocity, speed);
    }

    private void enableFeederWheel() {
        feederWheel.set(ControlMode.PercentOutput, 1);
    }

    private void disableFeederWheel() {
        feederWheel.set(ControlMode.PercentOutput, 0);
    }

    private void setHoodPosition(double desiredAngle) {
        // Finds difference between desired angle and actual angle and sets motor to travel that amount
        hoodPID.setReference((desiredAngle - getHoodPosition()) / 360, CANSparkMax.ControlType.kPosition);
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
