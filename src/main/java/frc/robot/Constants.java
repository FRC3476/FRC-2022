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

    /** Units are in Meters Per Second Squared*/
    public static final double MAX_ACCELERATION = 15; // TODO: Need to tune at field

    //field constants
    public static final Translation2d GOAL_POSITION = new Translation2d(1, 1); //TODO: get actual values

    // Shooter Constants

   public static final int SHOOTER_PERIOD_MS = 20;

    public static final int SHOOTER_WHEEL_CAN_MASTER_ID = 0; // TODO: Get actual CAN ID for all shooter components
    public static final int SHOOTER_WHEEL_CAN_SLAVE_ID = 0;
    public static final int FEEDER_WHEEL_CAN_ID = 0;
    public static final int HOOD_MOTOR_CAN_ID = 0;

    public static final int HOOD_ENCODER_DIO_ID = 0;
    public static final int HOOD_HOME_SWITCH_DIO_ID = 0;

    // Shooter PID
    // TODO: Configure PID for all shooter motors and current limits


    public static final int SHOOTER_PID_TIMEOUT_MS = 10;

    public static final int SHOOTER_P = 0;
    public static final int SHOOTER_I = 0;
    public static final int SHOOTER_D = 0;
    public static final int SHOOTER_F = 0;
    public static final int SHOOTER_I_ZONE = 0;
    public static final int SHOOTER_CURRENT_LIMIT_AMPS = 0;

    public static final int FEEDER_P = 0;
    public static final int FEEDER_I = 0;
    public static final int FEEDER_D = 0;
    public static final int FEEDER_F = 0;
    public static final int FEEDER_I_ZONE = 0;
    public static final int FEEDER_CURRENT_LIMIT_AMPS = 0;

    public static final int HOOD_P = 0;
    public static final int HOOD_I = 0;
    public static final int HOOD_D = 0;
    public static final int HOOD_F = 0;
    public static final int HOOD_I_ZONE = 0;
    public static final int HOOD_CURRENT_LIMIT_AMPS = 0;

    // Hood Constants

    public static final int HOOD_ABSOLUTE_ENCODER_OFFSET = 0;




}
