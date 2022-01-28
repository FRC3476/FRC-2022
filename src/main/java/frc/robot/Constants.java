package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public final class Constants {

    //Limelight 
    //Calibrate using https://www.desmos.com/calculator/n2dsvzsyhk
    public static final double CAMERA_TARGET_HEIGHT_OFFSET = 0; //TODO: CHANGE
    public static final double CAMERA_Y_ANGLE = 0; //TODO: CHANGE

    //Drive Constants
    public static final int DRIVE_PERIOD = 20; // TODO: APPEND UNITS

    //    public static final int DRIVE_LEFT_FRONT_ID = 10;
    //    public static final int DRIVE_LEFT_BACK_ID = 11;
    //    public static final int DRIVE_RIGHT_FRONT_ID = 12;
    //    public static final int DRIVE_RIGHT_BACK_ID = 13;
    //
    //    public static final int DRIVE_LEFT_FRONT_SWERVE_ID = 14;
    //    public static final int DRIVE_LEFT_BACK_SWERVE_ID = 15;
    //    public static final int DRIVE_RIGHT_FRONT_SWERVE_ID = 16;
    //    public static final int DRIVE_RIGHT_BACK_SWERVE_ID = 17;

    public static final int DRIVE_LEFT_FRONT_ID = 11;
    public static final int DRIVE_LEFT_BACK_ID = 13;
    public static final int DRIVE_RIGHT_FRONT_ID = 10;
    public static final int DRIVE_RIGHT_BACK_ID = 12;

    public static final int CAN_LEFT_FRONT_ID = 1;
    public static final int CAN_LEFT_BACK_ID = 3;
    public static final int CAN_RIGHT_FRONT_ID = 0;
    public static final int CAN_RIGHT_BACK_ID = 2;

    public static final int DRIVE_LEFT_FRONT_SWERVE_ID = 15;
    public static final int DRIVE_LEFT_BACK_SWERVE_ID = 17;
    public static final int DRIVE_RIGHT_FRONT_SWERVE_ID = 14;
    public static final int DRIVE_RIGHT_BACK_SWERVE_ID = 16;

    public static final double SWERVE_INCHES_PER_ROTATION = Math.PI;
    public static final double SWERVE_METER_PER_ROTATION = Units.inchesToMeters(SWERVE_INCHES_PER_ROTATION);
    public static final double SWERVE_DRIVE_P = 0.08;
    public static final double SWERVE_DRIVE_D = 0.00;
    public static final double SWERVE_DRIVE_I = 0.00;
    public static final double SWERVE_DRIVE_F = 0.00;

    /**
     * Feed forward constants for the drivetrain.
     * <p>
     * 0 -> Left Front
     * <p>
     * 1 -> Left Back
     * <p>
     * 2 -> Right Front
     * <p>
     * 3 -> Right Back
     */
    public static final SimpleMotorFeedforward[] DRIVE_FEEDFORWARD = {
            new SimpleMotorFeedforward(0.153, 1.6, 0.18),
            new SimpleMotorFeedforward(0.153, 1.6, 0.18),
            new SimpleMotorFeedforward(0.153, 1.6, 0.18),
            new SimpleMotorFeedforward(0.153, 1.6, 0.18)};

    /**
     * What the module states should be in hold mode. The wheels will be put in an X pattern to prevent the robot from moving.
     * <p>
     * 0 -> Left Front
     * <p>
     * 1 -> Left Back
     * <p>
     * 2 -> Right Front
     * <p>
     * 3 -> Right Back
     */
    public static final SwerveModuleState[] HOLD_MODULE_STATES = {
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
    };

    //    public static final Translation2d SWERVE_LEFT_FRONT_LOCATION = new Translation2d(-0.381, 0.381);
    //    public static final Translation2d SWERVE_LEFT_BACK_LOCATION = new Translation2d(-0.381, -0.381);
    //    public static final Translation2d SWERVE_RIGHT_FRONT_LOCATION = new Translation2d(0.381, 0.381);
    //    public static final Translation2d SWERVE_RIGHT_BACK_LOCATION = new Translation2d(0.381, -0.381);

    public static final Translation2d SWERVE_LEFT_FRONT_LOCATION = new Translation2d(-0.381, 0.381);
    public static final Translation2d SWERVE_LEFT_BACK_LOCATION = new Translation2d(-0.381, -0.381);
    public static final Translation2d SWERVE_RIGHT_FRONT_LOCATION = new Translation2d(0.381, 0.381);
    public static final Translation2d SWERVE_RIGHT_BACK_LOCATION = new Translation2d(0.381, -0.381);


    public static final double DRIVE_HIGH_SPEED_M = 7.26;
    @SuppressWarnings("unused") public static final double DRIVE_HIGH_SPEED_IN = Units.metersToInches(DRIVE_HIGH_SPEED_M);

    public static final double MAX_TURN_ERROR = 0.85;
    public static final double MAX_PID_STOP_SPEED = 5.2;

    /**
     * Units are in Meters Per Second Squared
     */
    public static final double MAX_ACCELERATION = 15; // TODO: Need to tune at field

    //field constants
    public static final Translation2d GOAL_POSITION = new Translation2d(1, 1); //TODO: get actual values

    // Shooter Constants
    
    public static final int SHOOTER_PERIOD_MS = 20;

    /**
     * Amount of Shooter Periods that have to elapse before 1 logging period is called
     */
    public static final int SHOOTER_PERIODS_PER_LOG = 3;

    public static final int SHOOTER_WHEEL_CAN_MASTER_ID = 0; // TODO: Get actual CAN ID for all shooter components
    public static final int SHOOTER_WHEEL_CAN_SLAVE_ID = 0;
    public static final int FEEDER_WHEEL_CAN_ID = 0;
    public static final int HOOD_MOTOR_CAN_ID = 0;

    public static final int HOOD_ENCODER_DIO_ID = 0;
    public static final int HOOD_HOME_SWITCH_DIO_ID = 0;

    // Shooter PID & Misc
    // TODO: Configure PID for all shooter motors and current limits

    /**
     * Timeout for shooter PIDs
     */
    public static final int SHOOTER_PID_TIMEOUT_MS = 10;

    public static final double SHOOTER_P = 0;
    public static final double SHOOTER_I = 0;
    public static final double SHOOTER_D = 0;

    /**
     * Shooter Flywheel Feed Forward for PID
     */
    public static final double SHOOTER_F = 0;

    /**
     * Integral Zone of shooter flywheel PID
     */
    public static final double SHOOTER_I_ZONE = 0;

    public static final double SHOOTER_CURRENT_LIMIT = 0;
    public static final double SHOOTER_TRIGGER_THRESHOLD_CURRENT = 0;
    public static final double SHOOTER_TRIGGER_THRESHOLD_TIME = 0;

    /**
     * Allowed Angular Speed error (in RPM) when comparing speed reported by encoder to an expected speed
     */
    public static final double ALLOWED_SHOOTER_SPEED_ERROR_RPM = 20;

    /**
     * Conversion from Falcon Sensor Units / 100ms to RPM 2048 is Sensor Units Per Revolution 600 Converts From Time of 100ms to 1
     * minute
     */
    public static final double FALCON_UNIT_CONVERSION_FOR_RELATIVE_ENCODER = 600 / 2048;


    public static final double FEEDER_WHEEL_P = 0;
    public static final double FEEDER_WHEEL_I = 0;
    public static final double FEEDER_WHEEL_D = 0;

    /**
     * Feed Forward for Feeder Wheel
     */
    public static final double FEEDER_WHEEL_F = 0;

    /**
     * Integral Zone for Feeder Wheel
     */
    public static final double FEEDER_WHEEL_I_ZONE = 0;

    public static final double FEEDER_CURRENT_LIMIT = 0;
    public static final double FEEDER_TRIGGER_THRESHOLD_CURRENT = 0;
    public static final double FEEDER_TRIGGER_THRESHOLD_TIME = 0;

    /**
     * The time that the feeder must be on before it is allowed to turn off
     */
    public static final double FEEDER_CHANGE_STATE_DELAY_SEC = .5;

    public static final double HOOD_P = 0;
    public static final double HOOD_I = 0;
    public static final double HOOD_D = 0;

    /**
     * Hood Feed Forward for PID
     */
    public static final double HOOD_F = 0;

    /**
     * Hood Integral Zone for PID
     */
    public static final double HOOD_I_ZONE = 0;

    public static final int HOOD_CURRENT_LIMIT_AMPS = 0;

    // Hood Constants

    /**
     * Offset for the absolute encoder on hood in order to make angle between 50 and 90
     */
    public static final double HOOD_ABSOLUTE_ENCODER_OFFSET = 0; // TODO: Find proper offset

    /**
     * Amount of degrees the hood turns per NEO550 rotation
     */
    public static final double HOOD_DEGREES_PER_MOTOR_ROTATION = 3.69230;

    /**
     * Maximum allowed time homing should take in seconds
     */
    public static final double MAX_HOMING_TIME_S = 2;

    /**
     * How many amps should be fed into Hood motor when HOMING
     */
    public static final double HOMING_MOTOR_CURRENT_AMPS = .05;

    /**
     * Allowed error when comparing Hood angle to a desired angle Units are in rotations of the motor. 1 Rotation is 3.69230
     * degrees of the hood
     */
    public static final double ALLOWED_HOOD_ANGLE_ERROR = .2;

    /**
     * If hood speed is under this value, hood has stopped
     */
    public static final double HOOD_HAS_STOPPED_REFERENCE = 1.0e-3;

    // Shooter Blinkin LED Constants

    // ON mode
    /**
     * Color value from -1 to 1 to be used with Blinkin LED
     * <p>
     * LED color is Red Orange - Solid
     */
    public static final double LED_FLYWHEEL_APPROACHING_DESIRED_SPEED = 0.63;

    /**
     * Color value from -1 to 1 to be used with Blinkin LED
     * <p>
     * LED color is Yellow - Solid
     */
    public static final double LED_HOOD_APPROACHING_DESIRED_POSITION = 0.69;

    /**
     * Color value from -1 to 1 to be used with Blinkin LED
     * <p>
     * LED color is Lime - Solid
     */
    public static final double LED_SHOOTER_READY_TO_SHOOT = 0.73;

    // HOMING
    /**
     * Color value from -1 to 1 to be used with Blinkin LED
     * <p>
     * Color is Blue - Strobe light
     */
    public static final double LED_HOOD_HOMING_IN_PROGRESS = -.09;

    // TEST
    /**
     * Color value from -1 to 1 to be used with Blinkin
     * <p>
     * LED Color is Gold - Strobe light
     */
    public static final double LED_TEST_IN_PROGRESS = -.07;

    // Shooter Test Constants

    /**
     * Shooter Speed in RPM that is used for TEST
     */
    public static final double SHOOTER_TEST_SPEED_RPM = 3000;

    /**
     * How long an individual test of a component should go on for in miliseconds
     */
    public static final long TEST_TIME_MS = 5000;


    //Climber Constants
    public static final int CLIMBER_PERIOD = 50;
    public static final int CLIMBER_MOTOR_ID = 20;
    public static final int CLIMBER_MOTOR_2_ID = 21;

    public static final double CLIMBER_MOTOR_KF = 0.0;
    public static final double CLIMBER_MOTOR_KP = 0.1;
    public static final double CLIMBER_MOTOR_KI = 0.0;
    public static final double CLIMBER_MOTOR_KD = 0.0;
    public static final double CLIMBER_MOTOR_IZONE = 10;
    public static final double CLIMBER_MOTOR_MAX_IACCUMULATOR = 0.1;
    public static final double CLIMBER_MOTOR_MAX_OUTPUT = 1.0;
    public static final int CLIMBER_MOTOR_MAX_ERROR = 5;

    public static final int ELEVATOR_ARM_CONTACT_SWITCH_A_DIO_CHANNEL = 0;
    public static final int ELEVATOR_ARM_CONTACT_SWITCH_B_DIO_CHANNEL = 1;

    public static final int PIVOTING_ARM_CONTACT_SWITCH_A_DIO_CHANNEL = 2;
    public static final int PIVOTING_ARM_CONTACT_SWITCH_B_DIO_CHANNEL = 3;

    public static final int PIVOTING_ARM_LATCHED_SWITCH_A_DIO_CHANNEL = 4;
    public static final int PIVOTING_ARM_LATCHED_SWITCH_B_DIO_CHANNEL = 5;

    public static final int LATCH_SOLENOID_ID = 1;
    public static final int PIVOT_SOLENOID_ID = 2;
    public static final int BRAKE_SOLENOID_ID = 3;

    /**
     * The height to go to once the drivers request the climber to deploy
     */
    public static final double CLIMBER_DEPLOY_HEIGHT = 10000;

    /**
     * If the elevator arm is below this height and going down, the climb will abort
     */
    public static final double MIN_CLIMBER_ELEVATOR_HEIGHT = 50;

    /**
     * If the elevator arm is above this height and going down, the climb will abort
     */
    public static final double MAX_CLIMBER_ELEVATOR_HEIGHT = 12000;

    /**
     * How long it takes for the pivot pneumatic to pivot open (become pivoted) (in seconds)
     */
    public static final double ARM_PIVOT_DURATION = 0.5;

    /**
     * How long it takes for the pivot pneumatic to close (become inline) (in seconds)
     */
    public static final double ARM_UNPIVOT_DURATION = 0.5;

    /**
     * How long it takes for the latch pneumatic on the pivot arm to unlatch (in seconds)
     */
    public static final double PIVOT_ARM_UNLATCH_DURATION = 0.5;

    /**
     * Amount (relative) to move the climber arm up to unlatch the elevator arm.
     */
    public static final double CLIMBER_ELEVATOR_UNLATCH_AMOUNT = 100;

    /**
     * The max safe height for the elevator arm during the swinging part of the climb
     */
    public static final double CLIMBER_ELEVATOR_MAX_SAFE_HEIGHT = 10000;

    /**
     * The height the elevator arm should be at when the climber is doing the final extension to hit the bar
     */
    public static final double MAX_CLIMBER_EXTENSION = 11000;

    //Robot Tracker
    public static final double SPARK_VELOCITY_MEASUREMENT_LATENCY = 0.112;
    public static final int ROBOT_TRACKER_PERIOD = 10;

    // Intake Constants TODO: Need To Set
    public static final int INTAKE_PERIOD = 50;
    public static final int SOLENOID_CHANNEL = 0;
    public static final int INTAKE_MOTOR_DEVICE_ID = 40;
    public static final double INTAKE_MOTOR_SPEED = 1.0;
    public static final double INTAKE_OPEN_TIME = 0.3;
}
