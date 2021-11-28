// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import java.util.Arrays;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANAnalog.AnalogMode;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.utility.ControllerDriveInputs;
import frc.utility.OrangeUtility;
import frc.utility.controllers.LazyCANSparkMax;

public class Drive extends AbstractSubsystem {

    public enum DriveState {
        TELEOP, TURN, HOLD, DONE, RAMSETE
    }

    private static final Drive instance = new Drive();

    public static Drive getInstance() {
        return instance;
    }

    private boolean drivePercentVbus;

    private AHRS gyroSensor;
    private final PIDController turnPID;
    private DriveState driveState;
    private Rotation2d wantedHeading = new Rotation2d();
    boolean rotateAuto = false;

    double prevPositionL = 0;
    double prevPositionR = 0;

    private boolean isAiming = false;

    double prevTime;

    private double turnTarget = 0;

    private final LazyCANSparkMax leftFrontSpark, leftBackSpark, rightFrontSpark, rightBackSpark;

    private final LazyCANSparkMax leftFrontSparkSwerve, leftBackSparkSwerve, rightFrontSparkSwerve,
            rightBackSparkSwerve;
    private final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(Constants.SWERVE_LEFT_FRONT_LOCATION, Constants.SWERVE_LEFT_BACK_LOCATION, 
    Constants.SWERVE_RIGHT_FRONT_LOCATION, Constants.SWERVE_RIGHT_BACK_LOCATION);
    /**
     * Motors that turn the wheels around
     */
    private final LazyCANSparkMax[] swerveMotors = new LazyCANSparkMax[4];

    /**
     * Motors that are driving the robot around and causing it to move
     */
    private final LazyCANSparkMax[] swerveDriveMotors = new LazyCANSparkMax[4];

    /**
     * Encoders for the motors that turn the wheel (NOT ABSOLUTE)
     */
    private final CANEncoder[] swerveEncoders = new CANEncoder[4];

    private final DutyCycle leftFrontSparkPwmEncoder, leftBackSparkPwmEncoder, rightFrontSparkPwmEncoder,
            rightBackSparkPwmEncoder;

    /**
     * Absolute Encoders for the motors that turn the wheel
     */
    private final DutyCycle[] swerveEncodersDIO = new DutyCycle[4];

    /**
     * PID Controllers for the swerve Drive
     */
    CANPIDController[] swervePID = new CANPIDController[4];



    private Drive() {
        super(Constants.DRIVE_PERIOD);
        gyroSensor = new AHRS(SPI.Port.kMXP);

        // Swerve Drive Motors
        leftFrontSpark = new LazyCANSparkMax(Constants.DRIVE_LEFT_FRONT_ID, MotorType.kBrushless);
        leftBackSpark = new LazyCANSparkMax(Constants.DRIVE_LEFT_BACK_ID, MotorType.kBrushless);
        rightFrontSpark = new LazyCANSparkMax(Constants.DRIVE_RIGHT_FRONT_ID, MotorType.kBrushless);
        rightBackSpark = new LazyCANSparkMax(Constants.DRIVE_RIGHT_BACK_ID, MotorType.kBrushless);

        leftFrontSpark.setInverted(false);
        rightFrontSpark.setInverted(false);
        leftBackSpark.setInverted(true);
        rightBackSpark.setInverted(true);

        leftFrontSparkSwerve = new LazyCANSparkMax(Constants.DRIVE_LEFT_FRONT_SWERVE_ID, MotorType.kBrushless);
        leftBackSparkSwerve = new LazyCANSparkMax(Constants.DRIVE_LEFT_BACK_SWERVE_ID, MotorType.kBrushless);
        rightFrontSparkSwerve = new LazyCANSparkMax(Constants.DRIVE_RIGHT_FRONT_SWERVE_ID, MotorType.kBrushless);
        rightBackSparkSwerve = new LazyCANSparkMax(Constants.DRIVE_RIGHT_BACK_SWERVE_ID, MotorType.kBrushless);

        leftFrontSparkPwmEncoder = new DutyCycle(new DigitalInput(0));
        leftBackSparkPwmEncoder = new DutyCycle(new DigitalInput(1));
        rightFrontSparkPwmEncoder = new DutyCycle(new DigitalInput(2));
        rightBackSparkPwmEncoder = new DutyCycle(new DigitalInput(3));

        swerveMotors[0] = leftFrontSparkSwerve;
        swerveMotors[1] = leftBackSparkSwerve;
        swerveMotors[2] = rightFrontSparkSwerve;
        swerveMotors[3] = rightBackSparkSwerve;

        swerveDriveMotors[0] = leftFrontSpark;
        swerveDriveMotors[1] = leftBackSpark;
        swerveDriveMotors[2] = rightFrontSpark;
        swerveDriveMotors[3] = rightBackSpark;

        swerveEncodersDIO[0] = leftFrontSparkPwmEncoder;
        swerveEncodersDIO[1] = leftBackSparkPwmEncoder;
        swerveEncodersDIO[2] = rightFrontSparkPwmEncoder;
        swerveEncodersDIO[3] = rightBackSparkPwmEncoder;

        for (int i = 0; i < 4; i++) {
            swerveEncoders[i] = swerveMotors[i].getEncoder();
            swerveEncoders[i].setPositionConversionFactor(8.1503);// 8.1466);
            
            swerveMotors[i].getAnalog(AnalogMode.kAbsolute).setPositionConversionFactor(360/3.3);//105.88);

            swervePID[i] = swerveMotors[i].getPIDController();
            swervePID[i].setP(Constants.SWERVE_DRIVE_P);
            swervePID[i].setD(Constants.SWERVE_DRIVE_D);
            swervePID[i].setI(Constants.SWERVE_DRIVE_I);
            swervePID[i].setFF(Constants.SWERVE_DRIVE_F);
            

            //Get data faster from the sparks
            swerveMotors[i].setPeriodicFramePeriod(PeriodicFrame.kStatus2, 15);
            swerveDriveMotors[i].setPeriodicFramePeriod(PeriodicFrame.kStatus2, 5);

            swerveMotors[i].setSmartCurrentLimit(15);
            swerveDriveMotors[i].setSmartCurrentLimit(30);

            swerveDriveMotors[i].setIdleMode(IdleMode.kBrake);
            swerveMotors[i].setIdleMode(IdleMode.kBrake);

            swerveDriveMotors[i].burnFlash();
            swerveMotors[i].burnFlash();
        }
    
        configMotors();

        drivePercentVbus = true;
        driveState = DriveState.TELEOP;

        turnPID = new PIDController(0.02, 0, 0.00, 0.02); //P=1.0 OR 0.8
        turnPID.disableContinuousInput();
        turnPID.setSetpoint(0);
        configBrake();
    }

    public SwerveDriveKinematics getSwerveDriveKinematics(){
        return swerveKinematics;
    }

    public void setDriveState(DriveState driveState){
        this.driveState = driveState;
    }

    private void configBrake() {
        //TODO
    }

    private void configCoast() {
        //TODO
    }

    private void configAuto() {
        
    }

    synchronized public void setTeleop() {
        driveState = DriveState.TELEOP;
    }

    synchronized public SwerveModuleState[] getSwerveModuleStates(){
        SwerveModuleState[] swerveModuleState = new SwerveModuleState[4];
        for(int i = 0; i<4; i++){
            SwerveModuleState moduleState = new SwerveModuleState(swerveDriveMotors[i].getEncoder().getVelocity()*Constants.SWERVE_METER_PER_ROTATION,
                    Rotation2d.fromDegrees(getAbsolutePosition(i)));
            swerveModuleState[i] = moduleState;
        }
        return swerveModuleState;
    }

    /**
     * The method gets the states of the swerve modules and then uses this information to return a chassis speeds
     * @return The current state of the robot as chassis speeds
     */
    public ChassisSpeeds getRobotState(){
        return swerveKinematics.toChassisSpeeds(getSwerveModuleStates());
    }

    public void calibrateGyro() {
        gyroSensor.calibrate();
    }



    public void startHold() {
        //TODO
        driveState = DriveState.HOLD;
    }

    public void endHold() {
        driveState = DriveState.TELEOP;
    }

    

    public void hold() {
        //TODO
        
    }

    public void swerveDrive(ControllerDriveInputs inputs){
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(Constants.DRIVE_HIGH_SPEED_M*inputs.getX(),
                Constants.DRIVE_HIGH_SPEED_M*inputs.getY(), 
                inputs.getRotation()*2);
        swerveDrive(chassisSpeeds);    
    }

    public void swerveDriveFieldRelative(ControllerDriveInputs inputs){
        double turnSpeed = 0;
        if(Math.abs(inputs.getRotation()) < 0.01){
            double error = turnTarget - getAngle();
            turnPID.setSetpoint(0);
            if(Math.abs(error) > 2) turnSpeed = turnPID.calculate(error);

            SmartDashboard.putNumber("gyro pid in", getGyroAngle().getDegrees() % 360);
            SmartDashboard.putNumber("pid Delta Speed", turnSpeed);
            SmartDashboard.putNumber("wanted heading", wantedHeading.getDegrees());
            SmartDashboard.putNumber("turn pid error", error);
        } else {
            turnSpeed = inputs.getRotation()*2;
            turnTarget = getAngle();
        }
        

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(Constants.DRIVE_HIGH_SPEED_M*inputs.getX(), 
                Constants.DRIVE_HIGH_SPEED_M*inputs.getY(), 
                turnSpeed, 
                Rotation2d.fromDegrees(-getAngle()));

        
        swerveDrive(chassisSpeeds);
    }

    double doubleMod(double x, double y){
        // x mod y behaving the same way as Math.floorMod but with doubles
        return (x - Math.floor(x/y) * y);
    }	

    private void swerveDrive(ChassisSpeeds chassisSpeeds){
        synchronized (this) {
            driveState = DriveState.TELEOP;
        }

        SmartDashboard.putNumber("Drive Command X Velocity", chassisSpeeds.vxMetersPerSecond);  
        SmartDashboard.putNumber("Drive Command Y Velocity", chassisSpeeds.vyMetersPerSecond);  
        SmartDashboard.putNumber("Drive Command Rotation", chassisSpeeds.omegaRadiansPerSecond); 

        SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        boolean rotate = true;

        if(chassisSpeeds.vxMetersPerSecond == 0 && chassisSpeeds.vyMetersPerSecond == 0 && chassisSpeeds.omegaRadiansPerSecond == 0) rotate = false;
        SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, Constants.DRIVE_HIGH_SPEED_M);

        for (int i = 0; i < 4; i++){
            SwerveModuleState targetState = SwerveModuleState.optimize(moduleStates[i], Rotation2d.fromDegrees(swerveEncoders[i].getPosition()));
            //SwerveModuleState targetState = moduleStates[i];
            double targetAngle = targetState.angle.getDegrees();
            double currentAngle = getAbsolutePosition(i); //swerveEncoders[i].getPosition();

            double angleDiff = doubleMod((targetAngle - currentAngle) + 180, 360) - 180;

            if(Math.abs(angleDiff) < 5 || !rotate){
                swerveMotors[i].set(0);
            }else{
                swervePID[i].setReference(swerveEncoders[i].getPosition() + angleDiff, ControlType.kPosition);
            }
            swerveDriveMotors[i].set(targetState.speedMetersPerSecond/Constants.DRIVE_HIGH_SPEED_M);

            SmartDashboard.putNumber("Swerve Motor " + i + " Target Position", swerveEncoders[i].getPosition() + angleDiff);
            SmartDashboard.putNumber("Swerve Motor " + i + " Error", angleDiff);           
        }
    }
    
    private void configMotors() {
        //TODO
        
    }

    /**
     * You probably want to get the angle from {@link RobotTracker#getPoseMeters()} 
     * @return
     */
    public double getAngle() {
        return gyroSensor.getAngle();
    }

    /**
     * You probably want to get the angle from {@link RobotTracker#getPoseMeters()} 
     * @return the rotation reported from the gyro
     */
    public Rotation2d getGyroAngle() {
        // -180 through 180
        return Rotation2d.fromDegrees(gyroSensor.getAngle());
    }

    /**
     * {@link Drive#getSpeed()} is faster if you don't need the square root
     * @return The robot speed
     */
    public double getSpeed() {
        ChassisSpeeds robotState = getRobotState();
        return Math.sqrt(Math.pow(robotState.vxMetersPerSecond, 2) + Math.pow(robotState.vyMetersPerSecond, 2));
    }
    /**
     * Should be a bit faster than {@link Drive#getSpeed()}
     * @return The robot speed squared
     */
    public double getSpeedSquared() {
        ChassisSpeeds robotState = getRobotState();
        return Math.pow(robotState.vxMetersPerSecond, 2) + Math.pow(robotState.vyMetersPerSecond, 2);
    }

    double autoStartTime;
    HolonomicDriveController controller = new HolonomicDriveController(
          new PIDController(1, 0, 0), new PIDController(1, 0, 0),
          new ProfiledPIDController(1, 0, 0,
        new TrapezoidProfile.Constraints(6.28, 3.14)));
    Trajectory currentAutoTrajectory;

    public synchronized void setAutoPath(Trajectory trajectory) {
        driveState = DriveState.RAMSETE;
        this.currentAutoTrajectory = trajectory;
        autoStartTime = Timer.getFPGATimestamp();
        configAuto();
        configCoast();
        updateRamsete();
    }

    private void updateRamsete() {
        Trajectory.State goal = currentAutoTrajectory.sample(Timer.getFPGATimestamp()-autoStartTime);
        System.out.println(goal);
        ChassisSpeeds adjustedSpeeds = controller.calculate(RobotTracker.getInstance().getPoseMeters(), goal, Rotation2d.fromDegrees(0));
        swerveDrive(adjustedSpeeds);
        //System.out.println(ramseteController.atReference());
        //System.out.println("target speed" + Units.metersToInches(wheelspeeds.leftMetersPerSecond) + " " + Units.metersToInches(wheelspeeds.rightMetersPerSecond) + "time: " +(Timer.getFPGATimestamp()-autoStartTime) );
        //TODO: not working
        if(controller.atReference() && (Timer.getFPGATimestamp()-autoStartTime)>= currentAutoTrajectory.getTotalTimeSeconds()){
            driveState = DriveState.DONE;
            stopMovement();
        }
    }

    public double getVoltage() {
        return 0;
        //return (leftTalon.getMotorOutputVoltage() + rightTalon.getMotorOutputVoltage()
        //		+ .getMotorOutputVoltage() + rightSlaveTalon.getMotorOutputVoltage()
        //		+ rightSlave2Talon.getMotorOutputVoltage() + leftSlave2Talon.getMotorOutputVoltage()) / 6;
    }	

    public synchronized void setSimpleDrive(boolean setting) {
        if(drivePercentVbus != setting) System.out.println("Simple drive: " + setting);
        drivePercentVbus = setting;
    }

    public synchronized boolean getSimpleDrive() {
        return drivePercentVbus;
    }

    @Override
    public void update() {
    //	System.out.println("L speed " + getLeftSpeed() + " position x " + RobotTracker.getInstance().getOdometry().translationMat.getX());
    //	System.out.println("R speed " + getRightSpeed() + " position y " + RobotTracker.getInstance().getOdometry().translationMat.getY());
    //debugSpeed();
    //System.out.println(driveState);	
    DriveState snapDriveState;
        synchronized (this) {
            snapDriveState = driveState;
        }
        switch (snapDriveState) {
            case TELEOP:				
                break;
            case TURN:
                updateTurn();
                break;
            case HOLD:
                hold();
                break;
            case DONE:
                break;
            case RAMSETE:
                updateRamsete();
                break;
                
        }
        
    }
    synchronized public boolean isAiming() {
        return isAiming; 
    }

    public void setRotation(Rotation2d angle) {
        synchronized (this) {
            wantedHeading = angle;
            driveState = DriveState.TURN;
            rotateAuto = true;
            isAiming = !getTurningDone();
            configBrake();
            
        }
    }

    public void setRotationTeleop(Rotation2d angle) {
        synchronized (this) {
            wantedHeading = angle;
            driveState = DriveState.TURN;
            rotateAuto = false;
            isAiming = !getTurningDone();
            configBrake();
            
        }
    }

    public synchronized boolean getTurningDone(){
        //TODO redo
        return false;
    }

    public synchronized void resetGyro(){
        gyroSensor.zeroYaw();
        wantedHeading = Rotation2d.fromDegrees(0);
        turnTarget = 0;
    }

    double turnMinSpeed = 0;

    private void updateTurn() {
        double error = wantedHeading.rotateBy(RobotTracker.getInstance().getGyroAngle()).getDegrees();
        double curSpeed =  Math.abs(Math.toDegrees(getRobotState().omegaRadiansPerSecond));
        double deltaSpeed;
        double pidDeltaSpeed;

        pidDeltaSpeed = turnPID.calculate(error);
        deltaSpeed = Math.copySign(Math.max(Math.abs(pidDeltaSpeed), turnMinSpeed), pidDeltaSpeed);
        
        
        if ((Math.abs(error) < Constants.MAX_TURN_ERROR) || curSpeed < Constants.MAX_PID_STOP_SPEED) {
            swerveDrive(new ChassisSpeeds(0, 0, Math.toRadians(0)));
            isAiming = false;
            
            if( rotateAuto ) {
                synchronized (this) {
                    configBrake();
                    driveState = DriveState.DONE;
                }
            }
            
        } else {
            System.out.println("Error: " + error + " curSpeed: " + curSpeed + " command: " + deltaSpeed + " pidOut: " 
                + pidDeltaSpeed + " minSpeed: " + turnMinSpeed);
            isAiming = true;
            swerveDrive(new ChassisSpeeds(0, 0, Math.toRadians(deltaSpeed)));

            if(curSpeed < 0.5) {
                //Updates every 10ms
                turnMinSpeed = Math.min(turnMinSpeed + 0.1, 6);
            } else {
                turnMinSpeed = 2;
            }
        }
    }

    synchronized public void stopMovement() {
        swerveDrive(new ChassisSpeeds(0, 0, 0));
    }

    synchronized public boolean isFinished() {
        return driveState == DriveState.DONE || driveState == DriveState.TELEOP;
    }

    @Override
    public void selfTest() {

    }

    @Override
    public void logData() {
        for(int i = 0; i < 4; i++){
            double relPos = swerveEncoders[i].getPosition() % 360;
            if(relPos < 0) relPos += 360;
            SmartDashboard.putNumber("Swerve Motor " + i + " Relative Position", relPos);
            SmartDashboard.putNumber("Swerve Motor " + i + " Absolute Position", getAbsolutePosition(i));
            SmartDashboard.putNumber("Drive Motor " + i + " Velocity", swerveDriveMotors[i].getEncoder().getVelocity());
            SmartDashboard.putNumber("Drive Motor " + i + " Current", swerveDriveMotors[i].getOutputCurrent());
            SmartDashboard.putNumber("Swerve Motor " + i + " Current", swerveMotors[i].getOutputCurrent());
        }

        ChassisSpeeds chassisSpeeds = getRobotState();
        SmartDashboard.putNumber("Computed Robot X Velocity", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Computed Robot Y Velocity", chassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Computed Robot Rotation", chassisSpeeds.omegaRadiansPerSecond);
    }


    public double getAbsolutePosition(int moduleNumber){
        return (1 - swerveEncodersDIO[moduleNumber].getOutput()) * 360;
    }
}
