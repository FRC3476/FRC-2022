// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
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
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.utility.ControllerDriveInputs;
import frc.utility.Timer;
import frc.utility.controllers.LazyTalonFX;
import frc.utility.wpimodified.HolonomicDriveController;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static frc.robot.Constants.*;


public final class Drive extends AbstractSubsystem {

    // PID TUNING
    final @NotNull NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();

    final @NotNull NetworkTableEntry turnP = SmartDashboard.getEntry("TurnPIDP");
    final @NotNull NetworkTableEntry turnI = SmartDashboard.getEntry("TurnPIDI");
    final @NotNull NetworkTableEntry turnD = SmartDashboard.getEntry("TurnPIDD");
    final @NotNull NetworkTableEntry turnMaxVelocity = SmartDashboard.getEntry("TurnMaxVelocity");
    final @NotNull NetworkTableEntry turnMaxAcceleration = SmartDashboard.getEntry("TurnMaxAcceleration");

    public void resetAuto() {
        ProfiledPIDController autoTurnPIDController
                = new ProfiledPIDController(8, 0, 0.01, new TrapezoidProfile.Constraints(4, 4));
        autoTurnPIDController.enableContinuousInput(-Math.PI, Math.PI);
        autoTurnPIDController.setTolerance(Math.toRadians(10));

        swerveAutoController = new HolonomicDriveController(
                new PIDController(3, 0, 0),
                new PIDController(3, 0, 0),
                autoTurnPIDController);
        swerveAutoController.setTolerance(new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(10))); //TODO: Tune
    }

    public enum DriveState {
        TELEOP, TURN, HOLD, DONE, RAMSETE, STOP
    }

    public boolean useRelativeEncoderPosition = false;

    private static final @NotNull Drive INSTANCE = new Drive();

    public static @NotNull Drive getInstance() {
        return INSTANCE;
    }

    private final @NotNull PIDController turnPID;

    {
        turnPID = new PIDController(DEFAULT_TURN_P, DEFAULT_TURN_I, DEFAULT_TURN_D); //P=1.0
        // OR 0.8
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    public @NotNull DriveState driveState;
    volatile Rotation2d wantedHeading = new Rotation2d();
    boolean rotateAuto = false;

    public boolean useFieldRelative;

    {
        logData("Drive Field Relative Allowed", true);
    }

    private boolean isAiming = false;

    private @NotNull final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            Constants.SWERVE_LEFT_FRONT_LOCATION,
            Constants.SWERVE_LEFT_BACK_LOCATION,
            Constants.SWERVE_RIGHT_FRONT_LOCATION,
            Constants.SWERVE_RIGHT_BACK_LOCATION
    );
    /**
     * Motors that turn the wheels around. Uses Falcon500s
     */
    final @NotNull LazyTalonFX[] swerveMotors = new LazyTalonFX[4];

    /**
     * Motors that are driving the robot around and causing it to move
     */
    final @NotNull LazyTalonFX[] swerveDriveMotors = new LazyTalonFX[4];

    /**
     * Absolute Encoders for the motors that turn the wheel
     */

    final @NotNull CANCoder[] swerveCanCoders = new CANCoder[4];

    private Drive() {
        super(Constants.DRIVE_PERIOD, 5);

        final @NotNull LazyTalonFX leftFrontTalon, leftBackTalon, rightFrontTalon, rightBackTalon;
        final @NotNull CANCoder leftFrontCanCoder, leftBackCanCoder, rightFrontCanCoder, rightBackCanCoder;
        final @NotNull LazyTalonFX leftFrontTalonSwerve, leftBackTalonSwerve, rightFrontTalonSwerve, rightBackTalonSwerve;
        // Swerve Drive Motors
        leftFrontTalon = new LazyTalonFX(Constants.DRIVE_LEFT_FRONT_ID);
        leftBackTalon = new LazyTalonFX(Constants.DRIVE_LEFT_BACK_ID);
        rightFrontTalon = new LazyTalonFX(Constants.DRIVE_RIGHT_FRONT_ID);
        rightBackTalon = new LazyTalonFX(Constants.DRIVE_RIGHT_BACK_ID);

        leftFrontTalon.setInverted(false);
        rightFrontTalon.setInverted(false);
        leftBackTalon.setInverted(false);
        rightBackTalon.setInverted(false);

        leftFrontTalonSwerve = new LazyTalonFX(Constants.DRIVE_LEFT_FRONT_SWERVE_ID);
        leftBackTalonSwerve = new LazyTalonFX(Constants.DRIVE_LEFT_BACK_SWERVE_ID);
        rightFrontTalonSwerve = new LazyTalonFX(Constants.DRIVE_RIGHT_FRONT_SWERVE_ID);
        rightBackTalonSwerve = new LazyTalonFX(Constants.DRIVE_RIGHT_BACK_SWERVE_ID);

        leftFrontCanCoder = new CANCoder(Constants.CAN_LEFT_FRONT_ID);
        leftBackCanCoder = new CANCoder(Constants.CAN_LEFT_BACK_ID);
        rightFrontCanCoder = new CANCoder(Constants.CAN_RIGHT_FRONT_ID);
        rightBackCanCoder = new CANCoder(Constants.CAN_RIGHT_BACK_ID);

        swerveMotors[0] = leftFrontTalonSwerve;
        swerveMotors[1] = leftBackTalonSwerve;
        swerveMotors[2] = rightFrontTalonSwerve;
        swerveMotors[3] = rightBackTalonSwerve;

        swerveDriveMotors[0] = leftFrontTalon;
        swerveDriveMotors[1] = leftBackTalon;
        swerveDriveMotors[2] = rightFrontTalon;
        swerveDriveMotors[3] = rightBackTalon;

        swerveCanCoders[0] = leftFrontCanCoder;
        swerveCanCoders[1] = leftBackCanCoder;
        swerveCanCoders[2] = rightFrontCanCoder;
        swerveCanCoders[3] = rightBackCanCoder;

        for (int i = 0; i < 4; i++) {
            // Sets swerveMotors PID
            swerveMotors[i].config_kP(0, Constants.SWERVE_DRIVE_P, Constants.SWERVE_MOTOR_PID_TIMEOUT_MS);
            swerveMotors[i].config_kD(0, Constants.SWERVE_DRIVE_D, Constants.SWERVE_MOTOR_PID_TIMEOUT_MS);
            swerveMotors[i].config_kI(0, Constants.SWERVE_DRIVE_I, Constants.SWERVE_MOTOR_PID_TIMEOUT_MS);
            swerveMotors[i].config_kF(0, Constants.SWERVE_DRIVE_F, Constants.SWERVE_MOTOR_PID_TIMEOUT_MS);
            swerveMotors[i].configClosedloopRamp(Constants.SWERVE_DRIVE_RAMP_RATE, Constants.SWERVE_MOTOR_PID_TIMEOUT_MS);
            swerveMotors[i].config_IntegralZone(0, Constants.SWERVE_DRIVE_INTEGRAL_ZONE);

            // Sets current limits for motors
            swerveMotors[i].configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
                    Constants.SWERVE_MOTOR_CURRENT_LIMIT, Constants.SWERVE_MOTOR_CURRENT_LIMIT, 0));

            swerveDriveMotors[i].configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
                    Constants.SWERVE_DRIVE_MOTOR_CURRENT_LIMIT, Constants.SWERVE_DRIVE_MOTOR_CURRENT_LIMIT, 0));

            swerveDriveMotors[i].configVoltageCompSaturation(Constants.SWERVE_DRIVE_VOLTAGE_LIMIT);

            // This makes motors brake when no RPM is set
            swerveDriveMotors[i].setNeutralMode(NeutralMode.Coast);
            swerveMotors[i].setNeutralMode(NeutralMode.Coast);
            swerveMotors[i].setInverted(true);
            swerveDriveMotors[i].configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_5Ms);
            swerveDriveMotors[i].setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 50);
            swerveDriveMotors[i].setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
            swerveMotors[i].setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 50);
            swerveMotors[i].setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);

            swerveCanCoders[i].setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 200);
            swerveCanCoders[i].setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20);
        }

        turnP.setDouble(Constants.DEFAULT_TURN_P);
        turnI.setDouble(Constants.DEFAULT_TURN_I);
        turnD.setDouble(Constants.DEFAULT_TURN_D);
        turnMaxVelocity.setDouble(DEFAULT_TURN_MAX_VELOCITY);
        turnMaxAcceleration.setDouble(DEFAULT_TURN_MAX_ACCELERATION);

        turnP.addListener(event -> turnPID.setP(event.getEntry().getDouble(Constants.DEFAULT_TURN_P)),
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        turnI.addListener(event -> turnPID.setI(event.getEntry().getDouble(Constants.DEFAULT_TURN_I)),
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        turnD.addListener(event -> turnPID.setD(event.getEntry().getDouble(Constants.DEFAULT_TURN_D)),
                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

//        turnMaxVelocity.addListener(event -> turnPID.setConstraints(
//                        new TrapezoidProfile.Constraints(turnMaxVelocity.getDouble(DEFAULT_TURN_MAX_VELOCITY),
//                                turnMaxAcceleration.getDouble(6))),
//                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
//
//        turnMaxAcceleration.addListener(
//                event -> turnPID.setConstraints(
//                        new TrapezoidProfile.Constraints(turnMaxVelocity.getDouble(DEFAULT_TURN_MAX_ACCELERATION),
//                                turnMaxAcceleration.getDouble(6))),
//                EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);


        useFieldRelative = true;

        driveState = DriveState.TELEOP;
    }
    
    public void configCoast() {
        for (LazyTalonFX swerveMotor : swerveMotors) {
            swerveMotor.setNeutralMode(NeutralMode.Coast);
        }

        for (LazyTalonFX swerveDriveMotor : swerveDriveMotors) {
            swerveDriveMotor.setNeutralMode(NeutralMode.Coast);
        }
    }

    public void configBrake() {
        for (LazyTalonFX swerveMotor : swerveMotors) {
            swerveMotor.setNeutralMode(NeutralMode.Brake);
        }

        for (LazyTalonFX swerveDriveMotor : swerveDriveMotors) {
            swerveDriveMotor.setNeutralMode(NeutralMode.Brake);
        }
    }

    /**
     * @return the relative position of the selected swerve drive motor
     */
    private double getRelativeSwervePosition(int motorNum) {
        return (swerveMotors[motorNum].getSelectedSensorPosition() / Constants.FALCON_ENCODER_TICKS_PER_ROTATIONS) *
                Constants.SWERVE_MOTOR_POSITION_CONVERSION_FACTOR * 360;
    }

    /**
     * Set the target position of the selected swerve motor
     *
     * @param motorNum the selected swerve motor
     * @param position the target position in degrees (0-360)
     */
    private void setSwerveMotorPosition(int motorNum, double position) {
        swerveMotors[motorNum].set(ControlMode.Position, ((position * Constants.FALCON_ENCODER_TICKS_PER_ROTATIONS) /
                Constants.SWERVE_MOTOR_POSITION_CONVERSION_FACTOR) / 360);
    }


    @SuppressWarnings("unused")
    private double getSwerveDrivePosition(int motorNum) {
        return (swerveDriveMotors[motorNum].getSelectedSensorPosition() / Constants.FALCON_ENCODER_TICKS_PER_ROTATIONS) * Constants.SWERVE_DRIVE_MOTOR_REDUCTION;
    }

    /**
     * @return Returns requested drive wheel velocity in RPM
     */
    private double getSwerveDriveVelocity(int motorNum) {
        return swerveDriveMotors[motorNum].getSelectedSensorVelocity() * Constants.FALCON_ENCODER_TICKS_PER_100_MS_TO_RPM * Constants.SWERVE_DRIVE_MOTOR_REDUCTION;
    }

    public @NotNull SwerveDriveKinematics getSwerveDriveKinematics() {
        return swerveKinematics;
    }

    public synchronized void setDriveState(@NotNull DriveState driveState) {
        this.driveState = driveState;
    }

    public void setTeleop() {
        setDriveState(DriveState.TELEOP);
    }

    synchronized public SwerveModuleState @NotNull [] getSwerveModuleStates() {
        SwerveModuleState[] swerveModuleState = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            SwerveModuleState moduleState = new SwerveModuleState(
                    ((getSwerveDriveVelocity(i) / 60) * Constants.SWERVE_METER_PER_ROTATION),
                    Rotation2d.fromDegrees(getWheelRotation(i)));
            swerveModuleState[i] = moduleState;
        }
        return swerveModuleState;
    }

    public void startHold() {
        setDriveState(DriveState.HOLD);
    }

    public void endHold() {
        setDriveState(DriveState.TELEOP);
    }

    public void doHold() {
        setSwerveModuleStates(Constants.HOLD_MODULE_STATES, true);
    }

    public void swerveDrive(@NotNull ControllerDriveInputs inputs) {

        setDriveState(DriveState.TELEOP);


        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(DRIVE_HIGH_SPEED_M * inputs.getX(),
                DRIVE_HIGH_SPEED_M * inputs.getY(),
                inputs.getRotation() * 7);
        swerveDrive(chassisSpeeds);
    }

    public void swerveDriveFieldRelative(@NotNull ControllerDriveInputs inputs) {
        setDriveState(DriveState.TELEOP);

        ChassisSpeeds chassisSpeeds;
        if (useFieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(DRIVE_HIGH_SPEED_M * inputs.getX(),
                    DRIVE_HIGH_SPEED_M * inputs.getY(),
                    inputs.getRotation() * 7,
                    RobotTracker.getInstance().getGyroAngle());
        } else {
            chassisSpeeds = new ChassisSpeeds(DRIVE_HIGH_SPEED_M * inputs.getX(),
                    DRIVE_HIGH_SPEED_M * inputs.getY(),
                    inputs.getRotation() * 7);
        }

        swerveDrive(chassisSpeeds);
    }

    public void swerveDrive(ChassisSpeeds chassisSpeeds) {

        chassisSpeeds = limitAcceleration(chassisSpeeds);


        SmartDashboard.putNumber("Drive Command X Velocity", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Drive Command Y Velocity", chassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Drive Command Rotation", chassisSpeeds.omegaRadiansPerSecond);

        SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(chassisSpeeds);

        boolean rotate = chassisSpeeds.vxMetersPerSecond != 0 ||
                chassisSpeeds.vyMetersPerSecond != 0 ||
                chassisSpeeds.omegaRadiansPerSecond != 0;

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DRIVE_HIGH_SPEED_M);
        setSwerveModuleStates(moduleStates, rotate);
    }

    public void setSwerveModuleStates(SwerveModuleState[] moduleStates, boolean rotate) {
        for (int i = 0; i < 4; i++) {
            SwerveModuleState targetState = SwerveModuleState.optimize(moduleStates[i],
                    Rotation2d.fromDegrees(getWheelRotation(i)));
            //SwerveModuleState targetState = moduleStates[i];
            // TODO: flip the acceleration if we flip the module
            double targetAngle = targetState.angle.getDegrees() % 360;
            if (targetAngle < 0) { // Make sure the angle is positive
                targetAngle += 360;
            }
            double currentAngle = getWheelRotation(i); //swerveEncoders[i].getPosition();

            double angleDiff = getAngleDiff(targetAngle, currentAngle);

            if (Math.abs(angleDiff) < 0.1 || !rotate) {
                swerveMotors[i].set(ControlMode.Velocity, 0);
            } else {
                setSwerveMotorPosition(i, getRelativeSwervePosition(i) + angleDiff);
            }

            double speedModifier = 1; //= 1 - (OrangeUtility.coercedNormalize(Math.abs(angleDiff), 5, 180, 0, 180) / 180);

            setMotorSpeed(i, targetState.speedMetersPerSecond * speedModifier, 0);

            SmartDashboard.putNumber("Swerve Motor " + i + " Speed Modifier", speedModifier);
            SmartDashboard.putNumber("Swerve Motor " + i + " Target Position", getRelativeSwervePosition(i) + angleDiff);
            SmartDashboard.putNumber("Swerve Motor " + i + " Error", angleDiff);
        }
    }

    /**
     * @param targetAngle  The angle we want to be at (0-360)
     * @param currentAngle The current angle of the module (0-360)
     * @return the shortest angle between the two angles
     */
    public double getAngleDiff(double targetAngle, double currentAngle) {
        double angleDiff = targetAngle - currentAngle;
        if (angleDiff > 180) {
            angleDiff -= 360;
        }

        if (angleDiff < -180) {
            angleDiff += 360;
        }
        return angleDiff;
    }

    @NotNull ChassisSpeeds lastRequestedVelocity = new ChassisSpeeds(0, 0, 0);

    private double lastLoopTime = 0;

    /**
     * Puts a limit on the acceleration. This method should be called before setting a chassis speeds to the robot drivebase.
     * <p>
     * Limits the acceleration by ensuring that the difference between the command and previous velocity doesn't exceed a set
     * value
     *
     * @param commandedVelocity Desired velocity (The chassis speeds is mutated to the limited acceleration)
     * @return an acceleration limited chassis speeds
     */
    @Contract(mutates = "param")
    @NotNull ChassisSpeeds limitAcceleration(@NotNull ChassisSpeeds commandedVelocity) {
        double dt;
        if ((Timer.getFPGATimestamp() - lastLoopTime) > ((double) Constants.DRIVE_PERIOD / 1000) * 20) {
            // If the dt is a lot greater than our nominal dt reset the acceleration limiting
            // (ex. we've been disabled for a while)
            lastRequestedVelocity = RobotTracker.getInstance().getLatencyCompedChassisSpeeds();
            dt = (double) Constants.DRIVE_PERIOD / 1000;
        } else {
            dt = Timer.getFPGATimestamp() - lastLoopTime;
        }
        lastLoopTime = Timer.getFPGATimestamp();

        double maxVelocityChange = Constants.MAX_ACCELERATION * dt;
        double maxAngularVelocityChange = Constants.MAX_ANGULAR_ACCELERATION * dt;

        Translation2d velocityCommand = new Translation2d(
                commandedVelocity.vxMetersPerSecond, commandedVelocity.vyMetersPerSecond
        );

        Translation2d lastVelocityCommand = new Translation2d(
                lastRequestedVelocity.vxMetersPerSecond, lastRequestedVelocity.vyMetersPerSecond
        );

        Translation2d velocityChange = velocityCommand.minus(lastVelocityCommand);
        double velocityChangeAngle = Math.atan2(velocityChange.getY(), velocityChange.getX()); //Radians

        // Check if velocity change exceeds max limit
        if (velocityChange.getNorm() > maxVelocityChange) {
            // Get limited velocity vector difference in cartesian coordinate system
            Translation2d limitedVelocityVectorChange = new Translation2d(maxVelocityChange, new Rotation2d(velocityChangeAngle));
            Translation2d limitedVelocityVector = lastVelocityCommand.plus(limitedVelocityVectorChange);

            commandedVelocity.vyMetersPerSecond = limitedVelocityVector.getX();
            commandedVelocity.vyMetersPerSecond = limitedVelocityVector.getY();
        }

        // Checks if requested change in Angular Velocity is greater than allowed
        if (Math.abs(commandedVelocity.omegaRadiansPerSecond - lastRequestedVelocity.omegaRadiansPerSecond)
                > maxAngularVelocityChange) {
            // Add the lastCommandVelocity and the maxAngularVelocityChange (changed to have the same sign as the actual change)
            commandedVelocity.omegaRadiansPerSecond =
                    lastRequestedVelocity.omegaRadiansPerSecond +
                            Math.copySign(maxAngularVelocityChange,
                                    commandedVelocity.omegaRadiansPerSecond - lastRequestedVelocity.omegaRadiansPerSecond);
        }

        lastRequestedVelocity = commandedVelocity; // save our current commanded velocity to be used in next iteration
        return commandedVelocity;
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
        // Converts ffv voltage to percent output and sets it to motor
        swerveDriveMotors[module].set(ControlMode.PercentOutput, ffv / Constants.SWERVE_DRIVE_VOLTAGE_LIMIT);
        SmartDashboard.putNumber("Out Volts " + module, ffv);
        //swerveDriveMotors[module].setVoltage(10 * velocity/Constants.SWERVE_METER_PER_ROTATION);
    }

    /**
     * {@link Drive#getSpeedSquared()} is faster if you don't need the square root
     *
     * @return The robot speed
     */
    public double getSpeed() {
        ChassisSpeeds robotState = RobotTracker.getInstance().getLatencyCompedChassisSpeeds();
        return Math.sqrt(Math.pow(robotState.vxMetersPerSecond, 2) + Math.pow(robotState.vyMetersPerSecond, 2));
    }

    /**
     * Set the robot's rotation when driving a path. This is meant to be used in the auto builder. Code that wants to use this
     * should use {@link Drive#setAutoRotation(Rotation2d)} instead.
     *
     * @param angle The angle to set the robot to
     */
    public void setAutoRotation(double angle) {
        System.out.println("setting angle " + angle);
        autoTargetHeading = Rotation2d.fromDegrees(angle);
    }

    /**
     * Should be a bit faster than {@link Drive#getSpeed()}
     *
     * @return The robot speed squared
     */
    public double getSpeedSquared() {
        ChassisSpeeds robotState = RobotTracker.getInstance().getLatencyCompedChassisSpeeds();
        return Math.pow(robotState.vxMetersPerSecond, 2) + Math.pow(robotState.vyMetersPerSecond, 2);
    }

    double autoStartTime;

    private @NotNull HolonomicDriveController swerveAutoController;

    public void setAutoPath(Trajectory trajectory) {
        currentAutoTrajectoryLock.lock();
        try {
            resetAuto();
            setDriveState(DriveState.RAMSETE);
            this.currentAutoTrajectory = trajectory;
            this.isAutoAiming = false;
            autoStartTime = Timer.getFPGATimestamp();
        } finally {
            currentAutoTrajectoryLock.unlock();
        }
    }

    Trajectory currentAutoTrajectory;
    final Lock currentAutoTrajectoryLock = new ReentrantLock();
    volatile Rotation2d autoTargetHeading;

    /**
     * Tells the robot to start aiming while driving the auto path
     */
    private volatile boolean isAutoAiming = false;

    public void setAutoAiming(boolean autoAiming) {
        isAutoAiming = autoAiming;
    }


    private void updateRamsete() {
        currentAutoTrajectoryLock.lock();
        try {
            Trajectory.State goal = currentAutoTrajectory.sample(Timer.getFPGATimestamp() - autoStartTime);

            Rotation2d targetHeading = autoTargetHeading;
            if (isAutoAiming) {
                targetHeading = VisionManager.getInstance().getAngleToTarget();
            }

            ChassisSpeeds adjustedSpeeds = swerveAutoController.calculate(
                    RobotTracker.getInstance().getLastEstimatedPoseMeters(),
                    goal,
                    targetHeading);

            swerveDrive(adjustedSpeeds);
            if (swerveAutoController.atReference() && (Timer.getFPGATimestamp() - autoStartTime) >= currentAutoTrajectory.getTotalTimeSeconds()) {
                setDriveState(DriveState.DONE);
                stopMovement();
            }
        } finally {
            currentAutoTrajectoryLock.unlock();
        }
    }

    public void setAutoRotation(@NotNull Rotation2d rotation) {
        autoTargetHeading = rotation;
        System.out.println("new rotation" + rotation.getDegrees());
    }

    public double getAutoElapsedTime() {
        return Timer.getFPGATimestamp() - autoStartTime;
    }

    @Override
    public void update() {
        DriveState snapDriveState;
        synchronized (this) {
            snapDriveState = driveState;
        }

        checkGyroConnection();

        switch (snapDriveState) {
            case TURN:
                updateTurn();
                break;
            case HOLD:
                doHold();
                break;
            case RAMSETE:
                updateRamsete();
                break;
            case STOP:
                swerveDrive(new ChassisSpeeds(0, 0, 0));
        }
    }

    synchronized public boolean isAiming() {
        return isAiming;
    }

    public synchronized void setRotation(Rotation2d angle) {
        wantedHeading = angle;
        driveState = DriveState.TURN;
        rotateAuto = true;
        isAiming = !isTurningDone();
    }

    public synchronized void setRotation(double angle) {
        setRotation(Rotation2d.fromDegrees(angle));
    }


    public synchronized boolean isTurningDone() {
        double error = wantedHeading.minus(RobotTracker.getInstance().getGyroAngle()).getDegrees();
        return (Math.abs(error) < Constants.MAX_TURN_ERROR);
    }

    double turnMinSpeed = 0;

    /**
     * Default method when the x and y velocity and the target heading are not passed
     */
    private void updateTurn() {
        updateTurn(new ControllerDriveInputs(0, 0, 0), wantedHeading, false, Math.toRadians(Constants.MAX_TURN_ERROR));
        // Field relative flag won't do anything since we're not moving
    }

    double lastTurnUpdate = 0;

    /**
     * This method takes in x and y velocity as well as the target heading to calculate how much the robot needs to turn in order
     * to face a target
     * <p>
     * xVelocity and yVelocity are in m/s
     *
     * @param controllerDriveInputs The x and y velocity of the robot (rotation is ignored)
     * @param targetHeading         The target heading the robot should face
     * @param useFieldRelative      Whether the target heading is field relative or robot relative
     */
    public void updateTurn(ControllerDriveInputs controllerDriveInputs, @NotNull Rotation2d targetHeading,
                           boolean useFieldRelative, double turnErrorRadians) {
        updateTurn(controllerDriveInputs, new State(targetHeading.getRadians(), 0), useFieldRelative, turnErrorRadians);
    }

    /**
     * This method takes in x and y velocity as well as the target heading to calculate how much the robot needs to turn in order
     * to face a target
     * <p>
     * xVelocity and yVelocity are in m/s
     *
     * @param controllerDriveInputs The x and y velocity of the robot (rotation is ignored)
     * @param goal                  The target state at the end of the turn (in radians, radians/s)
     * @param useFieldRelative      Whether the target heading is field relative or robot relative
     */
    public void updateTurn(ControllerDriveInputs controllerDriveInputs, State goal,
                           boolean useFieldRelative, double turnErrorRadians) {
        synchronized (this) {
            if (driveState != DriveState.TURN) setDriveState(DriveState.TELEOP);
        }


        turnPID.setSetpoint(goal.position);

        if (Timer.getFPGATimestamp() - 0.2 > lastTurnUpdate || turnPID.getPositionError() > Math.toRadians(30)) {
            turnPID.reset();
        }
        lastTurnUpdate = Timer.getFPGATimestamp();
        double pidDeltaSpeed = turnPID.calculate(RobotTracker.getInstance().getGyroAngle().getRadians());

//        System.out.println(
//                "turn error: " + Math.toDegrees(turnPID.getPositionError()) + " delta speed: " + Math.toDegrees(pidDeltaSpeed));
        double curSpeed = RobotTracker.getInstance().getLatencyCompedChassisSpeeds().omegaRadiansPerSecond;

        if (useFieldRelative) {
            swerveDrive(ChassisSpeeds.fromFieldRelativeSpeeds(
                    controllerDriveInputs.getX() * DRIVE_HIGH_SPEED_M * 0.45,
                    controllerDriveInputs.getY() * DRIVE_HIGH_SPEED_M * 0.45,
                    pidDeltaSpeed,
                    RobotTracker.getInstance().getGyroAngle()));
        } else {
            swerveDrive(new ChassisSpeeds(
                    controllerDriveInputs.getX() * DRIVE_HIGH_SPEED_M * 0.45,
                    controllerDriveInputs.getY() * DRIVE_HIGH_SPEED_M * 0.45,
                    pidDeltaSpeed));
        }

        if (Math.abs(goal.position - RobotTracker.getInstance().getGyroAngle().getRadians()) < turnErrorRadians) {
            synchronized (this) {
                isAiming = false;
                if (rotateAuto) {
                    this.driveState = DriveState.DONE;
                }
            }
        } else {
            synchronized (this) {
                isAiming = true;
            }
            if (curSpeed < 0.5) {
                //Updates every 20ms
                turnMinSpeed = Math.min(turnMinSpeed + 0.1, 6);
            } else {
                turnMinSpeed = 2;
            }
        }

        logData("Turn Position Error", Math.toDegrees(goal.position - RobotTracker.getInstance().getGyroAngle().getRadians()));
        logData("Turn Actual Speed", curSpeed);
        logData("Turn PID Command", pidDeltaSpeed);
        logData("Turn Min Speed", turnMinSpeed);
    }

    public void stopMovement() {
        System.out.println("In StopMovement");
        setDriveState(DriveState.STOP);
        System.out.println("Exiting StopMovement");
    }

    synchronized public boolean isFinished() {
        return driveState == DriveState.STOP || driveState == DriveState.DONE || driveState == DriveState.TELEOP;
    }

    @Override
    public void selfTest() {

    }

    @Override
    public void logData() {
        for (int i = 0; i < 4; i++) {
            double relPos = getRelativeSwervePosition(i) % 360;
            if (relPos < 0) relPos += 360;
            logData("Swerve Motor " + i + " Relative Position", relPos);
            logData("Swerve Motor " + i + " Absolute Position", getWheelRotation(i));
            logData("Drive Motor " + i + " Velocity", getSwerveDriveVelocity(i) / 60.0d);
            logData("Drive Motor " + i + " Current", swerveDriveMotors[i].getStatorCurrent());
            logData("Swerve Motor " + i + " Current", swerveMotors[i].getStatorCurrent());
        }

        logData("Drive State", driveState.toString());
    }


    /**
     * Returns the angle/position of the requested encoder module
     *
     * @param moduleNumber the module to set
     * @return angle in degrees of the module (0-360)
     */
    public double getWheelRotation(int moduleNumber) {
        if (useRelativeEncoderPosition) {
            double relPos = getRelativeSwervePosition(moduleNumber) % 360;
            if (relPos < 0) relPos += 360;
            return relPos;
        } else {
            return swerveCanCoders[moduleNumber].getAbsolutePosition();
        }
    }

    /**
     * Returns the angle/position of the modules
     *
     * @return and array of the four angles
     */
    public double[] getWheelRotations() {
        double[] positions = new double[4];
        for (int i = 0; i < 4; i++) {
            if (useRelativeEncoderPosition) {
                double relPos = getRelativeSwervePosition(i) % 360;
                if (relPos < 0) relPos += 360;
                positions[i] = relPos;
            } else {
                positions[i] = swerveCanCoders[i].getAbsolutePosition();
            }
        }
        return positions;
    }


    @Contract(pure = true)
    public double[] getModuleSpeeds() {
        double[] speeds = new double[4];

        for (int i = 0; i < 4; i++) {
            speeds[i] = (getSwerveDriveVelocity(i) / 60.0d) * Constants.SWERVE_METER_PER_ROTATION;
        }
        return speeds;
    }

    /**
     * Checks if gyro is connected. If disconnected, switches to robot-centric drive for the rest of the match. Reports error to
     * driver station when this happens.
     */
    public void checkGyroConnection() {
        if (!RobotTracker.getInstance().getGyro().isConnected()) {
            if (useFieldRelative) {
                useFieldRelative = false;
                logData("Drive Field Relative Allowed", false);
                DriverStation.reportError("Gyro disconnected, switching to non field relative drive for rest of match", false);
            }
        }
    }

    public void setAbsoluteZeros() {
        for (int i = 0; i < swerveCanCoders.length; i++) {
            CANCoder swerveCanCoder = swerveCanCoders[i];
            System.out.println(i + " Setting Zero " + swerveCanCoder.configGetMagnetOffset() + " -> 0");
            swerveCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
            swerveCanCoder.configMagnetOffset(-(swerveCanCoder.getAbsolutePosition() - swerveCanCoder.configGetMagnetOffset()));
        }
    }

    /**
     * Closing of Shooter motors is not supported.
     */
    @Override
    public void close() throws UnsupportedOperationException {
        throw new UnsupportedOperationException("Can not close this object");
    }

    public void turnToAngle(double degrees) throws InterruptedException {
        System.out.println("Turning to " + degrees);
        setRotation(degrees);
        while (!isTurningDone()) {
            Thread.onSpinWait();
            if (Thread.interrupted()) throw new InterruptedException("Interrupted while turning");
        }
    }
}
