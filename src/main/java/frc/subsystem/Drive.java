// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.utility.ControllerDriveInputs;
import frc.utility.controllers.LazyCANSparkMax;

import java.lang.reflect.Field;


public final class Drive extends AbstractSubsystem {

    public enum DriveState {
        TELEOP, TURN, HOLD, DONE, RAMSETE
    }

    private static Drive instance = new Drive();

    public static Drive getInstance() {
        return instance;
    }

    private final AHRS gyroSensor;
    private final PIDController turnPID;
    private DriveState driveState;
    private Rotation2d wantedHeading = new Rotation2d();
    boolean rotateAuto = false;

    private boolean isAiming = false;

    private double turnTarget = 0;

    private final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(Constants.SWERVE_LEFT_FRONT_LOCATION,
            Constants.SWERVE_LEFT_BACK_LOCATION, Constants.SWERVE_RIGHT_FRONT_LOCATION, Constants.SWERVE_RIGHT_BACK_LOCATION);
    /**
     * Motors that turn the wheels around
     */
    final LazyCANSparkMax[] swerveMotors = new LazyCANSparkMax[4];

    /**
     * Motors that are driving the robot around and causing it to move
     */
    final LazyCANSparkMax[] swerveDriveMotors = new LazyCANSparkMax[4];

    /**
     * Encoders for the motors that turn the wheel (NOT ABSOLUTE)
     */
    final RelativeEncoder[] swerveEncoders = new RelativeEncoder[4];

    /**
     * Absolute Encoders for the motors that turn the wheel
     */
    final DutyCycle[] swerveEncodersDIO = new DutyCycle[4];

    /**
     * PID Controllers for the swerve Drive
     */
    final SparkMaxPIDController[] swervePID = new SparkMaxPIDController[4];

    private Drive() {
        super(Constants.DRIVE_PERIOD);
        gyroSensor = new AHRS(SPI.Port.kMXP);

        final LazyCANSparkMax leftFrontSpark, leftBackSpark, rightFrontSpark, rightBackSpark;
        final DutyCycle leftFrontSparkPwmEncoder, leftBackSparkPwmEncoder, rightFrontSparkPwmEncoder, rightBackSparkPwmEncoder;
        final LazyCANSparkMax leftFrontSparkSwerve, leftBackSparkSwerve, rightFrontSparkSwerve, rightBackSparkSwerve;
        // Swerve Drive Motors
        leftFrontSpark = new LazyCANSparkMax(Constants.DRIVE_LEFT_FRONT_ID, MotorType.kBrushless);
        leftBackSpark = new LazyCANSparkMax(Constants.DRIVE_LEFT_BACK_ID, MotorType.kBrushless);
        rightFrontSpark = new LazyCANSparkMax(Constants.DRIVE_RIGHT_FRONT_ID, MotorType.kBrushless);
        rightBackSpark = new LazyCANSparkMax(Constants.DRIVE_RIGHT_BACK_ID, MotorType.kBrushless);

        leftFrontSpark.setInverted(false);
        rightFrontSpark.setInverted(false);
        leftBackSpark.setInverted(false);
        rightBackSpark.setInverted(false);

        leftFrontSparkSwerve = new LazyCANSparkMax(Constants.DRIVE_LEFT_FRONT_SWERVE_ID, MotorType.kBrushless);
        leftBackSparkSwerve = new LazyCANSparkMax(Constants.DRIVE_LEFT_BACK_SWERVE_ID, MotorType.kBrushless);
        rightFrontSparkSwerve = new LazyCANSparkMax(Constants.DRIVE_RIGHT_FRONT_SWERVE_ID, MotorType.kBrushless);
        rightBackSparkSwerve = new LazyCANSparkMax(Constants.DRIVE_RIGHT_BACK_SWERVE_ID, MotorType.kBrushless);

        leftFrontSparkPwmEncoder = new DutyCycle(new DigitalInput(1));
        leftBackSparkPwmEncoder = new DutyCycle(new DigitalInput(3));
        rightFrontSparkPwmEncoder = new DutyCycle(new DigitalInput(0));
        rightBackSparkPwmEncoder = new DutyCycle(new DigitalInput(2));

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
            swerveDriveMotors[i].getEncoder().setPositionConversionFactor(1);
            swerveDriveMotors[i].getEncoder().setVelocityConversionFactor(1);

            swerveMotors[i].getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute).setPositionConversionFactor(360 / 3.3);//105.88);

            swervePID[i] = swerveMotors[i].getPIDController();
            swervePID[i].setP(Constants.SWERVE_DRIVE_P);
            swervePID[i].setD(Constants.SWERVE_DRIVE_D);
            swervePID[i].setI(Constants.SWERVE_DRIVE_I);
            swervePID[i].setFF(Constants.SWERVE_DRIVE_F);


            //Get data faster from the sparks
            swerveMotors[i].setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);
            swerveDriveMotors[i].setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);

            swerveMotors[i].setSmartCurrentLimit(15);
            swerveDriveMotors[i].setSmartCurrentLimit(30);

            swerveDriveMotors[i].setIdleMode(IdleMode.kBrake);
            swerveMotors[i].setIdleMode(IdleMode.kBrake);
            swerveDriveMotors[i].burnFlash();
            swerveMotors[i].burnFlash();
        }

        configMotors();
        driveState = DriveState.TELEOP;

        turnPID = new PIDController(0.02, 0.01, 0.00, 0.02); //P=1.0 OR 0.8
        turnPID.disableContinuousInput();
        turnPID.setSetpoint(0);
        configBrake();
    }

    public SwerveDriveKinematics getSwerveDriveKinematics() {
        return swerveKinematics;
    }

    public void setDriveState(DriveState driveState) {
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

    synchronized public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] swerveModuleState = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            SwerveModuleState moduleState = new SwerveModuleState(
                    (swerveDriveMotors[i].getEncoder().getVelocity() / 60d) * Constants.SWERVE_METER_PER_ROTATION,
                    Rotation2d.fromDegrees(getAbsolutePosition(i)));
            swerveModuleState[i] = moduleState;
        }
        return swerveModuleState;
    }

    /**
     * The method gets the states of the swerve modules and then uses this information to return a chassis speeds
     *
     * @return The current state of the robot as chassis speeds
     */
    public ChassisSpeeds getRobotState() {
        return swerveKinematics.toChassisSpeeds(getSwerveModuleStates());
    }

    public void calibrateGyro() {
        gyroSensor.calibrate();
    }


    public void startHold() {
        configBrake();
        driveState = DriveState.HOLD;
    }

    public void endHold() {
        driveState = DriveState.TELEOP;
    }

    public void doHold() {
        setSwerveModuleStates(Constants.HOLD_MODULE_STATES);
    }

    public void swerveDrive(ControllerDriveInputs inputs) {
        synchronized (this) {
            driveState = DriveState.TELEOP;
        }

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(Constants.DRIVE_HIGH_SPEED_M * inputs.getX(),
                Constants.DRIVE_HIGH_SPEED_M * inputs.getY(),
                inputs.getRotation() * 2);
        swerveDrive(chassisSpeeds);
    }

    public void swerveDriveFieldRelative(ControllerDriveInputs inputs) {
        synchronized (this) {
            driveState = DriveState.TELEOP;
        }

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(Constants.DRIVE_HIGH_SPEED_M * inputs.getX(),
                Constants.DRIVE_HIGH_SPEED_M * inputs.getY(),
                inputs.getRotation() * 6,
                Rotation2d.fromDegrees(getAngle()));

        if (chassisSpeeds.vxMetersPerSecond == 0 && chassisSpeeds.vyMetersPerSecond == 0 && chassisSpeeds.omegaRadiansPerSecond == 0) {
            // We're not moving, so put the robot in a hold pose to prevent us from moving when pushed
            doHold();
        } else {
            swerveDrive(chassisSpeeds);
        }
    }

    double doubleMod(double x, double y) {
        // x mod y behaving the same way as Math.floorMod but with doubles
        return (x - Math.floor(x / y) * y);
    }

    public void swerveDrive(ChassisSpeeds chassisSpeeds) {
        swerveDrive(chassisSpeeds, 0);
    }

    public void swerveDrive(ChassisSpeeds chassisSpeeds, double acceleration) {
        SmartDashboard.putNumber("Drive Command X Velocity", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Drive Command Y Velocity", chassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Drive Command Rotation", chassisSpeeds.omegaRadiansPerSecond);

        SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        boolean rotate = chassisSpeeds.vxMetersPerSecond != 0 ||
                chassisSpeeds.vyMetersPerSecond != 0 ||
                chassisSpeeds.omegaRadiansPerSecond != 0;

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.DRIVE_HIGH_SPEED_M);
        setSwerveModuleStates(moduleStates, rotate, acceleration);
    }

    public void setSwerveModuleStates(SwerveModuleState[] states) {
        setSwerveModuleStates(states, true, 0);
    }

    public void setSwerveModuleStates(SwerveModuleState[] moduleStates, boolean rotate, double acceleration) {
        for (int i = 0; i < 4; i++) {
            //            SwerveModuleState targetState = SwerveModuleState.optimize(moduleStates[i],
            //                    Rotation2d.fromDegrees(getAbsolutePosition(i)));
            // TODO: flip the acceleration if we flip the module
            SwerveModuleState targetState = moduleStates[i];
            double targetAngle = targetState.angle.getDegrees();
            double currentAngle = getAbsolutePosition(i); //swerveEncoders[i].getPosition();

            double angleDiff = doubleMod((targetAngle - currentAngle) + 180, 360) - 180;

            if (Math.abs(angleDiff) < 5 || !rotate) {
                swerveMotors[i].set(0);
            } else {
                swervePID[i].setReference(swerveEncoders[i].getPosition() + angleDiff, CANSparkMax.ControlType.kPosition);
            }

            double speedModifier = 1; //= 1 - (OrangeUtility.coercedNormalize(Math.abs(angleDiff), 5, 180, 0, 180) / 180);

            setMotorSpeed(i, targetState.speedMetersPerSecond * speedModifier, acceleration);

            SmartDashboard.putNumber("Swerve Motor " + i + " Speed Modifier", speedModifier);
            SmartDashboard.putNumber("Swerve Motor " + i + " Target Position", swerveEncoders[i].getPosition() + angleDiff);
            SmartDashboard.putNumber("Swerve Motor " + i + " Error", angleDiff);
        }
    }

    /**
     * Sets the motor voltage
     *
     * @param module       The module to set the voltage on
     * @param velocity     The target velocity
     * @param acceleration The acceleration to use
     */
    public void setMotorSpeed(int module, double velocity, double acceleration) {
        double ffv = Constants.DRIVE_FEEDFORWARD[module].calculate(velocity, acceleration);
        swerveDriveMotors[module].setVoltage(ffv);
        SmartDashboard.putNumber("Out Volts " + module, ffv);
        //swerveDriveMotors[module].setVoltage(10 * velocity/Constants.SWERVE_METER_PER_ROTATION);
    }

    private void configMotors() {
        //TODO

    }

    /**
     * You probably want to get the angle from {@link RobotTracker#getPoseMeters()}
     *
     * @return the current angle of the robot in degrees
     */
    public double getAngle() {
        return -gyroSensor.getAngle();
    }

    /**
     * You probably want to get the angle from {@link RobotTracker#getPoseMeters()}
     *
     * @return the rotation reported from the gyro
     */
    public Rotation2d getGyroAngle() {
        // -180 through 180
        return Rotation2d.fromDegrees(-getAngle());
    }

    /**
     * {@link Drive#getSpeedSquared()} is faster if you don't need the square root
     *
     * @return The robot speed
     */
    public double getSpeed() {
        ChassisSpeeds robotState = getRobotState();
        return Math.sqrt(Math.pow(robotState.vxMetersPerSecond, 2) + Math.pow(robotState.vyMetersPerSecond, 2));
    }

    /**
     * Should be a bit faster than {@link Drive#getSpeed()}
     *
     * @return The robot speed squared
     */
    public double getSpeedSquared() {
        ChassisSpeeds robotState = getRobotState();
        return Math.pow(robotState.vxMetersPerSecond, 2) + Math.pow(robotState.vyMetersPerSecond, 2);
    }

    double autoStartTime;
    private final HolonomicDriveController controller = new HolonomicDriveController(
            new PIDController(1.5, 0, 0),
            new PIDController(1.5, 0, 0),
            new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(6, 5)));

    {
        controller.setTolerance(new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(10))); //TODO: Tune
    }


    public synchronized void setAutoPath(Trajectory trajectory) {
        driveState = DriveState.RAMSETE;
        this.currentAutoTrajectory = trajectory;
        autoStartTime = Timer.getFPGATimestamp();
        configAuto();
        configCoast();
    }

    Trajectory currentAutoTrajectory;
    Rotation2d autoTargetHeading;

    private void updateRamsete() {
        Trajectory.State goal = currentAutoTrajectory.sample(Timer.getFPGATimestamp() - autoStartTime);
        System.out.println(goal);
        ChassisSpeeds adjustedSpeeds = controller.calculate(RobotTracker.getInstance().getPoseMeters(), goal,
                autoTargetHeading);
        swerveDrive(adjustedSpeeds, goal.accelerationMetersPerSecondSq);
        //System.out.println(ramseteController.atReference());
        //System.out.println("target speed" + Units.metersToInches(wheelspeeds.leftMetersPerSecond) + " " + Units
        // .metersToInches(wheelspeeds.rightMetersPerSecond) + "time: " +(Timer.getFPGATimestamp()-autoStartTime) );
        //TODO: not working
        if (controller.atReference() && (Timer.getFPGATimestamp() - autoStartTime) >= currentAutoTrajectory.getTotalTimeSeconds()) {
            driveState = DriveState.DONE;
            stopMovement();
        }
    }

    public synchronized void setAutoRotation(Rotation2d rotation) {
        autoTargetHeading = rotation;
        System.out.println("new rotation" + rotation.getDegrees());
    }

    public double getAutoElapsedTime() {
        return Timer.getFPGATimestamp() - autoStartTime;
    }

    @Override
    public void update() {
        //	System.out.println("L speed " + getLeftSpeed() + " position x " + RobotTracker.getInstance().getOdometry()
        //	.translationMat.getX());
        //	System.out.println("R speed " + getRightSpeed() + " position y " + RobotTracker.getInstance().getOdometry()
        //	.translationMat.getY());
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
                doHold();
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


    public synchronized boolean getTurningDone() {
        //TODO redo
        return false;
    }

    public synchronized void resetGyro() {
        gyroSensor.zeroYaw();
        wantedHeading = Rotation2d.fromDegrees(0);
        turnTarget = 0;
    }

    double turnMinSpeed = 0;

    private void updateTurn() {
        double error = wantedHeading.rotateBy(RobotTracker.getInstance().getGyroAngle()).getDegrees();
        double pidDeltaSpeed = turnPID.calculate(error);
        double curSpeed = Math.toDegrees(getRobotState().omegaRadiansPerSecond);
        double deltaSpeed = Math.copySign(Math.max(Math.abs(pidDeltaSpeed), turnMinSpeed), pidDeltaSpeed);


        if ((Math.abs(error) < Constants.MAX_TURN_ERROR) && curSpeed < Constants.MAX_PID_STOP_SPEED) {
            swerveDrive(new ChassisSpeeds(0, 0, Math.toRadians(0)));
            isAiming = false;

            if (rotateAuto) {
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

            if (curSpeed < 0.5) {
                //Updates every 10ms
                turnMinSpeed = Math.min(turnMinSpeed + 0.1, 6);
            } else {
                turnMinSpeed = 2;
            }
            SmartDashboard.putNumber("Turn Error", error);
            SmartDashboard.putNumber("Turn Actual Speed", curSpeed);
            SmartDashboard.putNumber("Turn PID Command", pidDeltaSpeed);
            SmartDashboard.putNumber("Turn Speed Command", deltaSpeed);
            SmartDashboard.putNumber("Turn Min Speed", turnMinSpeed);


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
        for (int i = 0; i < 4; i++) {
            double relPos = swerveEncoders[i].getPosition() % 360;
            if (relPos < 0) relPos += 360;
            SmartDashboard.putNumber("Swerve Motor " + i + " Relative Position", relPos);
            SmartDashboard.putNumber("Swerve Motor " + i + " Absolute Position", getAbsolutePosition(i));
            SmartDashboard.putNumber("Drive Motor " + i + " Velocity", swerveDriveMotors[i].getEncoder().getVelocity() / 60d);
            SmartDashboard.putNumber("Drive Motor " + i + " Current", swerveDriveMotors[i].getOutputCurrent());
            SmartDashboard.putNumber("Swerve Motor " + i + " Current", swerveMotors[i].getOutputCurrent());

        }

        ChassisSpeeds chassisSpeeds = getRobotState();
        SmartDashboard.putNumber("Computed Robot X Velocity", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Computed Robot Y Velocity", chassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Computed Robot Rotation", chassisSpeeds.omegaRadiansPerSecond);
    }


    public double getAbsolutePosition(int moduleNumber) {
        double angle = ((1 - swerveEncodersDIO[moduleNumber].getOutput()) * 360) - 90;
        if (moduleNumber == 3) angle -= 72;
        return angle < 0 ? angle + 360 : angle;
    }

    public void fallbackAim(Translation2d currentPos) {
        Translation2d diff = Constants.GOAL_POSITION.minus(currentPos);
        Rotation2d rotation = new Rotation2d(diff.getX(), diff.getY());
        setRotation(rotation);
    }

    @Override
    public void close() throws Exception {
        Field digitalSource = DutyCycle.class.getDeclaredField("m_source");
        digitalSource.setAccessible(true);

        for (int i = 0; i < 4; i++) {
            swerveDriveMotors[i].close();
            swerveMotors[i].close();

            DigitalSource ds = (DigitalSource) digitalSource.get(swerveEncodersDIO[i]);
            ds.close(); // Use reflection to close the DigitalSource because the DIO class doesn't do it properly.
            swerveEncodersDIO[i].close();
        }
        gyroSensor.close();
        instance = new Drive();
    }
}
