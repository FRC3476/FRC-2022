package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public final class Constants {

    // Logging Period
    /**
     * Every subsystem will log one for every 20 update periods
     */
    public static final int DEFAULT_PERIODS_PER_LOG = 20;
    // Input Constants
    /**
     * This is the max time in seconds that a driver can let go of a button that is supposed to be held and still have it count as
     * holding the button
     */
    public static final double MAX_TIME_NOT_HELD_SEC = 0.100;

    /**
     * Time required in seconds for a button that is supposed to be held down to activate
     */
    public static final double HELD_BUTTON_TIME_THRESHOLD_SEC = 1;

    //Limelight
    //Calibrate using https://www.desmos.com/calculator/n2dsvzsyhk
    public static final double CAMERA_TARGET_HEIGHT_OFFSET = 62.1657; //TODO: CHANGE
    public static final double CAMERA_Y_ANGLE = 35.3203; //TODO: CHANGE

    // Vision Manager
    public static final int VISION_MANAGER_PERIOD = 50; //22Hz
    public static final double SHOOT_TIME_PER_BALL = 0.2; // For Auto

    /**
     * Max speed of the robot while shooter (m/s)
     */
    public static final double MAX_SHOOT_SPEED = 0.05;
    /**
     * Relative position of the limelight from the center of the robot.
     */
    public static final Translation2d LIMELIGHT_CENTER_OFFSET = new Translation2d(-0.684, 0); //TODO: CHANGE
    public static final double VISION_MANAGER_DISTANCE_THRESHOLD_SQUARED = Math.pow(1.0, 2); //TODO: CHANGE

    //Drive Constants
    public static final int DRIVE_PERIOD = 20;

    public static final int DRIVE_LEFT_FRONT_ID = 11;
    public static final int DRIVE_LEFT_BACK_ID = 12;
    public static final int DRIVE_RIGHT_FRONT_ID = 13;
    public static final int DRIVE_RIGHT_BACK_ID = 14;

    public static final int DRIVE_LEFT_FRONT_SWERVE_ID = 15;
    public static final int DRIVE_LEFT_BACK_SWERVE_ID = 16;
    public static final int DRIVE_RIGHT_FRONT_SWERVE_ID = 17;
    public static final int DRIVE_RIGHT_BACK_SWERVE_ID = 18;

    public static final int CAN_LEFT_FRONT_ID = 19;
    public static final int CAN_LEFT_BACK_ID = 20;
    public static final int CAN_RIGHT_FRONT_ID = 21;
    public static final int CAN_RIGHT_BACK_ID = 22;

    public static final double SWERVE_INCHES_PER_ROTATION = 12.5 * 0.976;
    public static final double SWERVE_METER_PER_ROTATION = Units.inchesToMeters(SWERVE_INCHES_PER_ROTATION);
    public static final double SWERVE_DRIVE_P = 0.1;
    public static final double SWERVE_DRIVE_D = 0.00;
    public static final double SWERVE_DRIVE_I = 0.00;
    public static final double SWERVE_DRIVE_F = 0.00;
    public static final double SWERVE_DRIVE_INTEGRAL_ZONE = 0.00;
    public static final double SWERVE_DRIVE_RAMP_RATE = 0.1;

    public static final int SWERVE_MOTOR_PID_TIMEOUT_MS = 50;

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
            new SimpleMotorFeedforward(0.17763, 2.7731, 0.55827),
            new SimpleMotorFeedforward(0.17763, 2.7731, 0.55827),
            new SimpleMotorFeedforward(0.17763, 2.7731, 0.55827),
            new SimpleMotorFeedforward(0.17763, 2.7731, 0.55827)};


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
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45))
    };
    /**
     * What the module states should be set to when we start climbing. All the wheels will face forward to make the robot easy to
     * push once it is disabled.
     * <p>
     * 0 -> Left Front
     * <p>
     * 1 -> Left Back
     * <p>
     * 2 -> Right Front
     * <p>
     * 3 -> Right Back
     */
    public static final SwerveModuleState[] SWERVE_MODULE_STATE_FORWARD = {
            new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(90))
    };

    // 0.307975 is 12.125 in inches
    public static final Translation2d SWERVE_LEFT_FRONT_LOCATION = new Translation2d(0.307975, 0.307975);
    public static final Translation2d SWERVE_LEFT_BACK_LOCATION = new Translation2d(-0.307975, 0.307975);
    public static final Translation2d SWERVE_RIGHT_FRONT_LOCATION = new Translation2d(0.307975, -0.307975);
    public static final Translation2d SWERVE_RIGHT_BACK_LOCATION = new Translation2d(-0.307975, -0.307975);


    public static final double DRIVE_HIGH_SPEED_M = 4.2;
    @SuppressWarnings("unused") public static final double DRIVE_HIGH_SPEED_IN = Units.metersToInches(DRIVE_HIGH_SPEED_M);

    /**
     * Allowed Turn Error in degrees.
     */
    public static final double MAX_TURN_ERROR = 1.5;

    /**
     * Allowed Turn Error in degrees.
     */
    public static final double MAX_PID_STOP_SPEED = 100;

    // 2048 sensor units per revolution
    public static final double FALCON_ENCODER_TICKS_PER_ROTATIONS = 2048;
    public static final double FALCON_ENCODER_TICKS_PER_100_MS_TO_RPM = 600 / 2048.0d;
    public static final double SWERVE_MOTOR_POSITION_CONVERSION_FACTOR = 1 / 12.8;

    public static final int SWERVE_MOTOR_CURRENT_LIMIT = 15;
    public static final int SWERVE_DRIVE_MOTOR_CURRENT_LIMIT = 15;
    public static final int SWERVE_DRIVE_VOLTAGE_LIMIT = 12;

    public static final double SWERVE_DRIVE_MOTOR_REDUCTION = 1 / 8.14;

    /**
     * Units are in Meters Per Second Squared Supposed to be 5
     */
    public static final double MAX_ACCELERATION = 25;

    /**
     * Units are in Radians per Second Squared
     */
    public static final double MAX_ANGULAR_ACCELERATION = Math.toRadians(360 * 27);

    //field/Vision Manager constants
    public static final Translation2d GOAL_POSITION = new Translation2d(8.25, 0);
    public static final double VISION_PREDICT_AHEAD_TIME = 0.5;

    public static final double GOAL_RADIUS_IN = 24.0;
    /**
     * The distance to the center of the goal to the vision tape.
     */
    public static final double GOAL_RADIUS = Units.inchesToMeters(GOAL_RADIUS_IN);

    /**
     * This GOAL RADIUS is used to calculate the turn error
     */
    public static final double GOAL_RADIUS_TURN_ERROR_M = GOAL_RADIUS * 3;

    /**
     * Maximum speed of the robot when shooting the ball. (Only applies when doing the static shot) Units are in Meters per Second
     * Squared
     */
    public static final double MAX_SHOOT_SPEED_SQUARED = Math.pow(MAX_SHOOT_SPEED, 2);

    public static final double GRAVITY = 9.80665;

    /**
     * Goal height in meters.
     */
    public static final double GOAL_HEIGHT = 2.64;
    /**
     * The height of the center of the Ejection point in meters.
     */
    public static final double SHOOTER_HEIGHT = 0.5; //TODO: Config

    public static final double MAX_SHOOTER_RPM = 5500;
    public static final double MAX_PREFER_SHOOTER_RPM = 4500;


    //Hopper Constants
    public static final int HOPPER_PERIOD = 200;
    public static final double HOPPER_SPEED = 1;
    public static final int HOPPER_MOTOR_ID = 30;
    public static final int HOPPER_CURRENT_LIMIT = 25;


    // Shooter Constants

    public static final int SHOOTER_PERIOD_MS = 50;

    public static final int SHOOTER_WHEEL_CAN_MASTER_ID = 50;
    public static final int SHOOTER_WHEEL_CAN_SLAVE_ID = 51;
    public static final int FEEDER_WHEEL_CAN_ID = 52;
    public static final int HOOD_MOTOR_CAN_ID = 53;
    public static final int HOOD_ABSOLUTE_ENCODER_CAN_ID = 54;

    public static final int HOOD_HOME_SWITCH_DIO_ID = 6;

    // Shooter PID & Misc
    // TODO: Configure PID for all shooter motors and current limits

    public static final double SHOOTER_P = 3.0e-4; //0.00074361;
    public static final double SHOOTER_I = 0.001;
    public static final double SHOOTER_D = 0;
    public static final double SHOOTER_F = 0.000068 * 1023;
    public static final double SHOOTER_I_ZONE = 500 / FALCON_ENCODER_TICKS_PER_100_MS_TO_RPM;

    public static final double SHOOTER_CURRENT_LIMIT = 40;
    public static final double SHOOTER_TRIGGER_THRESHOLD_CURRENT = 40;
    public static final double SHOOTER_TRIGGER_THRESHOLD_TIME = 0;

    /**
     * Allowed Angular Speed error (in RPM) when comparing speed reported by encoder to an expected speed
     */
    public static final double ALLOWED_SHOOTER_SPEED_ERROR_RPM = 300;

    /**
     * Conversion from Falcon Sensor Units / 100ms to RPM 2048 is Sensor Units Per Revolution 600 Converts From Time of 100ms to 1
     * minute
     */
    public static final double FALCON_UNIT_CONVERSION_FOR_RELATIVE_ENCODER = 600.0d / 2048.0d;

    public static final double SET_SHOOTER_SPEED_CONVERSION_FACTOR = (2048.0d / 600.0d) * (30.0 / 31.0);

    public static final double SET_SHOOTER_SPEED_CONVERSION_FACTOR_FEEDER = (2048.0d / 600.0d) * (2d);

    public static final double FEEDER_WHEEL_LOCK_SPEED_RPM = 1000;

    // Feeder wheel pidf is unused
    public static final double FEEDER_WHEEL_P = 0.0002;
    public static final double FEEDER_WHEEL_I = 0;
    public static final double FEEDER_WHEEL_D = 0;
    public static final double FEEDER_WHEEL_F = 0;
    public static final double FEEDER_WHEEL_I_ZONE = 0;

    public static final double FEEDER_WHEEL_SPEED = 1.0;

    public static final double FEEDER_CURRENT_LIMIT = 30;
    public static final double FEEDER_TRIGGER_THRESHOLD_CURRENT = 30;
    public static final double FEEDER_TRIGGER_THRESHOLD_TIME = 0.1;

    /**
     * The time that the feeder must be on before it is allowed to turn off
     */
    public static final double FEEDER_CHANGE_STATE_DELAY_SEC = 0.08;

    public static final double HOOD_P = 0.03;
    public static final double HOOD_I = 0.0005;
    public static final double HOOD_D = 0;
    public static final double HOOD_F = 0;
    public static final double HOOD_I_ZONE = 2.5;

    public static final double HOOD_MAX_OUTPUT = 0.5;

    public static final int HOOD_CURRENT_LIMIT_AMPS = 20;
    public static final double SHOOTER_HOMING_CURRENT_LIMIT = 10;

    // Hood Constants

    /**
     * Amount of degrees the hood turns per NEO550 rotation
     */
    public static final double HOOD_DEGREES_PER_MOTOR_ROTATION = 3.69230;

    /**
     * Maximum allowed time homing should take in seconds
     */
    public static final double MAX_HOMING_TIME_S = 2;

    /**
     * 30 Percent of motor power should be used when homing
     */
    public static final double HOMING_MOTOR_PERCENT_OUTPUT = 0.30;

    /**
     * Allowed error when comparing Hood angle to a desired angle Units are in degrees
     */
    public static final double ALLOWED_HOOD_ANGLE_ERROR = 2;

    /**
     * If _ speed is under this value, hood has stopped
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
    public static final double LED_HOOD_HOMING_IN_PROGRESS = -0.09;

    // TEST
    /**
     * Color value from -1 to 1 to be used with Blinkin
     * <p>
     * LED Color is Gold - Strobe light
     */
    public static final double LED_TEST_IN_PROGRESS = -0.07;

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
    public static final int CLIMBER_MOTOR_ID = 25;
    public static final int CLIMBER_MOTOR_2_ID = 26;

    public static final double CLIMBER_MOTOR_KF = 0.0;
    public static final double CLIMBER_MOTOR_KP = 0.1;
    public static final double CLIMBER_MOTOR_KI = 0.0;
    public static final double CLIMBER_MOTOR_KD = 0.0;
    public static final double CLIMBER_MOTOR_IZONE = 10;
    public static final double CLIMBER_MOTOR_MAX_IACCUMULATOR = 0.1;
    public static final double CLIMBER_MOTOR_MAX_OUTPUT = 0.3;
    public static final int CLIMBER_MOTOR_MAX_ERROR = 5;

    public static final int CLIMBER_CURRENT_LIMIT = 30;

    public static final int ELEVATOR_ARM_CONTACT_SWITCH_A_DIO_CHANNEL = 0;
    public static final int ELEVATOR_ARM_CONTACT_SWITCH_B_DIO_CHANNEL = 1;

    public static final int PIVOTING_ARM_CONTACT_SWITCH_A_DIO_CHANNEL = 2;
    public static final int PIVOTING_ARM_CONTACT_SWITCH_B_DIO_CHANNEL = 3;

    public static final int PIVOTING_ARM_LATCHED_SWITCH_A_DIO_CHANNEL = 4;
    public static final int PIVOTING_ARM_LATCHED_SWITCH_B_DIO_CHANNEL = 5;

    public static final int LATCH_SOLENOID_ID = 1;
    public static final int PIVOT_SOLENOID_ID = 0;
    public static final int BRAKE_SOLENOID_ID = 2;


    public static final double CLIMBER_ENCODER_TICKS_PER_INCH = 2048 * ((68.0 / 9.0) * (36.0 / 20.0)) / (12 * (3.0 / 8.0));
    /**
     * The height to go to once the drivers request the climber to deploy
     */
    public static final double CLIMBER_DEPLOY_HEIGHT = 23.45625 * CLIMBER_ENCODER_TICKS_PER_INCH;

    /**
     * If the elevator arm is below this height and going down, the climb will abort
     */

    public static final double MIN_CLIMBER_ELEVATOR_HEIGHT = -0.5 * CLIMBER_ENCODER_TICKS_PER_INCH;

    /**
     * If the elevator arm is above this height and going down, the climb will abort
     */

    public static final double MAX_CLIMBER_ELEVATOR_HEIGHT = 28.125 * CLIMBER_ENCODER_TICKS_PER_INCH;


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
    public static final double CLIMBER_ELEVATOR_MAX_SAFE_HEIGHT = 17.3475 * CLIMBER_ENCODER_TICKS_PER_INCH;

    /**
     * The height the elevator arm should be at when the climber is doing the final extension to hit the bar
     */
    public static final double MAX_CLIMBER_EXTENSION = 26 * CLIMBER_ENCODER_TICKS_PER_INCH;

    /**
     * Length to grab onto mid bar
     */
    public static final double CLIMBER_GRAB_ON_FIRST_BAR_EXTENSION = 0 * CLIMBER_ENCODER_TICKS_PER_INCH;

    /**
     * Length to grab on high and traversal bars
     */
    public static final double CLIMBER_GRAB_ON_NEXT_BAR_EXTENSION = 26.331 * CLIMBER_ENCODER_TICKS_PER_INCH;

    /**
     * How long only one of the sensor switches can be closed for before the climb will pause
     */
    public static final double MAX_ALLOW_ONLY_ONE_SWITCH_CONTACT_TIME = 0.1;

    //Robot Tracker
    public static final double DRIVE_VELOCITY_MEASUREMENT_LATENCY = 0.0025;
    public static final int ROBOT_TRACKER_PERIOD = 20;

    public static final int INTAKE_PERIOD = 50;
    public static final int INTAKE_SOLENOID_CHANNEL = 3;
    public static final int INTAKE_MOTOR_DEVICE_ID = 40;
    public static final double INTAKE_MOTOR_SPEED = -1;
    public static final double INTAKE_OPEN_TIME = 0.3;
}
