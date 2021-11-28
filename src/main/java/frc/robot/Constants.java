package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;

public class Constants {

    //Limelight 
    //Calibrate using https://www.desmos.com/calculator/n2dsvzsyhk
    public static final double CAMERA_TARGET_HEIGHT_OFFSET = 0; //TODO: CHANGE
    public static final double CAMERA_Y_ANGLE = 0; //TODO: CHANGE

    //Drive Constants
    public static final int DRIVE_PERIOD = 10;

    public static final int DRIVE_LEFT_FRONT_ID = 10;
    public static final int DRIVE_LEFT_BACK_ID =  11;
    public static final int DRIVE_RIGHT_FRONT_ID = 12;
    public static final int DRIVE_RIGHT_BACK_ID = 13;

    public static final int DRIVE_LEFT_FRONT_SWERVE_ID = 14;
    public static final int DRIVE_LEFT_BACK_SWERVE_ID = 15;
    public static final int DRIVE_RIGHT_FRONT_SWERVE_ID = 16;
    public static final int DRIVE_RIGHT_BACK_SWERVE_ID = 17;
    
    public static final double SWERVE_INCHES_PER_ROTATION = 4.12507923d/6d;
    public static final double SWERVE_METER_PER_ROTATION = Units.inchesToMeters(SWERVE_INCHES_PER_ROTATION);
    public static final double SWERVE_DRIVE_P = 0.08;
    public static final double SWERVE_DRIVE_D = 0.00;
    public static final double SWERVE_DRIVE_I = 0.00;
    public static final double SWERVE_DRIVE_F = 0.00;
    
    public static final Translation2d SWERVE_LEFT_FRONT_LOCATION = new Translation2d(-0.381, 0.381);
    public static final Translation2d SWERVE_LEFT_BACK_LOCATION = new Translation2d(-0.381, -0.381);
    public static final Translation2d SWERVE_RIGHT_FRONT_LOCATION = new Translation2d(0.381, 0.381);
    public static final Translation2d SWERVE_RIGHT_BACK_LOCATION = new Translation2d(0.381, -0.381);
    
    public static final double DRIVE_HIGH_SPEED_IN = 145;
    public static final double DRIVE_HIGH_SPEED_M = Units.inchesToMeters(DRIVE_HIGH_SPEED_IN);
    
    public static final double MAX_TURN_ERROR = 0.85;
    public static final double MAX_PID_STOP_SPEED = 5.2;
}
