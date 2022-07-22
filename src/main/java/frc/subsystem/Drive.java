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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.utility.ControllerDriveInputs;
import frc.utility.Timer;
import frc.utility.controllers.LazyTalonFX;
import frc.utility.geometry.MutableTranslation2d;
import frc.utility.wpimodified.HolonomicDriveController;
import frc.utility.wpimodified.PIDController;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static frc.robot.Constants.*;
import static frc.utility.MathUtil.*;
import static frc.utility.geometry.GeometryUtils.angleOf;


public final class Drive extends AbstractSubsystem {
    final @NotNull NetworkTableEntry turnP = SmartDashboard.getEntry("TurnPIDP");
    final @NotNull NetworkTableEntry turnI = SmartDashboard.getEntry("TurnPIDI");
    final @NotNull NetworkTableEntry turnD = SmartDashboard.getEntry("TurnPIDD");
    final @NotNull NetworkTableEntry turnMaxVelocity = SmartDashboard.getEntry("TurnMaxVelocity");
    final @NotNull NetworkTableEntry turnMaxAcceleration = SmartDashboard.getEntry("TurnMaxAcceleration");

    public void resetAuto() {
        swerveAutoControllerLock.lock();
        try {
            ProfiledPIDController autoTurnPIDController
                    = new ProfiledPIDController(8, 0, 0.01, new TrapezoidProfile.Constraints(4, 4));
            autoTurnPIDController.enableContinuousInput(-Math.PI, Math.PI);
            autoTurnPIDController.setTolerance(Math.toRadians(10));

            swerveAutoController = new HolonomicDriveController(
                    new edu.wpi.first.math.controller.PIDController(3, 0, 0),
                    new edu.wpi.first.math.controller.PIDController(3, 0, 0),
                    autoTurnPIDController);
            swerveAutoController.setTolerance(new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(10))); //TODO: Tune
        } finally {
            swerveAutoControllerLock.unlock();
        }
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
        turnPID.setIntegratorRange(-Math.PI * 2 * 4, Math.PI * 2 * 4);
    }

    public @NotNull DriveState driveState;
    volatile Rotation2d wantedHeading = new Rotation2d();
    boolean rotateAuto = false;

    public boolean useFieldRelative;

    {
        logData("Drive Field Relative Allowed", true);
    }

    private boolean isAiming = false;

    private @NotNull static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
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

    public volatile @NotNull Constants.AccelerationLimits accelerationLimit = AccelerationLimits.NORMAL_DRIVING;

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
            swerveMotors[i].configMotionAcceleration(Constants.SWERVE_ACCELERATION, Constants.SWERVE_MOTOR_PID_TIMEOUT_MS);
            swerveMotors[i].configMotionCruiseVelocity(Constants.SWERVE_CRUISE_VELOCITY, Constants.SWERVE_MOTOR_PID_TIMEOUT_MS);
            swerveMotors[i].config_IntegralZone(0, Constants.SWERVE_DRIVE_INTEGRAL_ZONE);

            // Sets current limits for motors
            swerveMotors[i].configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
                    Constants.SWERVE_MOTOR_CURRENT_LIMIT, Constants.SWERVE_MOTOR_CURRENT_LIMIT, 0));

            swerveDriveMotors[i].configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
                    Constants.SWERVE_DRIVE_MOTOR_CURRENT_LIMIT, Constants.SWERVE_DRIVE_MOTOR_CURRENT_LIMIT, 0));

            swerveDriveMotors[i].configVoltageCompSaturation(Constants.SWERVE_DRIVE_VOLTAGE_LIMIT, 100);


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
        //turnMaxAcceleration.setDouble(DEFAULT_TURN_MAX_ACCELERATION);

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
        swerveMotors[motorNum].set(ControlMode.MotionMagic, ((position * Constants.FALCON_ENCODER_TICKS_PER_ROTATIONS) /
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

    public static @NotNull SwerveDriveKinematics getSwerveDriveKinematics() {
        return SWERVE_DRIVE_KINEMATICS;
    }

    public synchronized void setDriveState(@NotNull DriveState driveState) {
        this.driveState = driveState;
    }

    public void setTeleop() {
        setDriveState(DriveState.TELEOP);
    }

    public @NotNull SwerveModuleState[] getSwerveModuleStates() {
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

    public void swerveDrive(@NotNull ControllerDriveInputs inputs, boolean useFieldRelative) {
        Translation2d targetVelocity = new Translation2d(inputs.getX(), inputs.getY()).times(DRIVE_HIGH_SPEED_M);
        double targetRotationSpeed = inputs.getRotation() * MAX_DRIVE_TURN_SPEED;
        if (this.useFieldRelative && useFieldRelative) {
            swerveDriveFieldRelative(targetVelocity, targetRotationSpeed, null);
        } else {
            swerveDrive(targetVelocity, targetRotationSpeed, null);
        }
    }

    public void swerveDriveFieldRelative(@Nullable Translation2d targetFieldRelativeSpeed,
                                         double targetAngularSpeed,
                                         @Nullable Translation2d additionalFieldRelativeAcceleration) {
        Rotation2d gyroAngle = RobotTracker.getInstance().getGyroAngle().unaryMinus();
        Translation2d targetRobotRelativeSpeed = null;
        if (targetFieldRelativeSpeed != null) {
            targetRobotRelativeSpeed = targetFieldRelativeSpeed.rotateBy(gyroAngle);
        }

        Translation2d additionalRobotRelativeAcceleration = null;
        if (additionalFieldRelativeAcceleration != null) {
            additionalRobotRelativeAcceleration = additionalFieldRelativeAcceleration.rotateBy(gyroAngle);
        }

        swerveDrive(targetRobotRelativeSpeed, targetAngularSpeed, additionalRobotRelativeAcceleration);
    }

    private @NotNull Translation2d expectedAcceleration = new Translation2d(0, 0);

    /**
     * @param targetRobotRelativeSpeed The target speed of the robot in meters per second. If null, the robot will assume that you
     *                                 only want to control the robot via the acceleration.
     * @param targetAngularSpeed       The target angular speed of the robot in radians per second
     */
    @Contract(mutates = "this")
    public void swerveDrive(@Nullable Translation2d targetRobotRelativeSpeed,
                            double targetAngularSpeed,
                            @Nullable Translation2d additionalRobotRelativeAcceleration) {
        final ChassisSpeeds currentSpeeds = SWERVE_DRIVE_KINEMATICS.toChassisSpeeds(getSwerveModuleStates());

        final Translation2d currentTranslationalVelocity =
                new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

        final Translation2d neededVelocityChange = targetRobotRelativeSpeed != null ?
                targetRobotRelativeSpeed.minus(currentTranslationalVelocity) : new Translation2d(0, 0);


        // Create two different accelerations with:
        //     lowRobotAcceleration = translational acceleration which will result in the least voltage used for translating
        //     highRobotAcceleration = translational acceleration which will result in the most voltage used for translating

        // This acceleration will slow down the robot as quickly as possible, but don't overshoot a velocity of 0.
        // Also ensures that don't use extra voltage to decelerate.
        final double currentTranslationalVelocityNorm = currentTranslationalVelocity.getNorm();
        double maxDeceleration = min(accelerationLimit.turningTranslationDeceleration,
                currentTranslationalVelocityNorm / (DRIVE_PERIOD / 1000.0),
                getMaxAccelerationToAvoidTipping(angleOf(currentTranslationalVelocity)),
                (currentTranslationalVelocityNorm * DRIVE_FEEDFORWARD.kv) / DRIVE_FEEDFORWARD.ka);
        MutableTranslation2d lowRobotAcceleration = new MutableTranslation2d(currentTranslationalVelocity).
                normalize().times(maxDeceleration).unaryMinus();

        // Acceleration to reach the target velocity as quickly as possible
        double maxAcceleration = min(accelerationLimit.acceleration,
                neededVelocityChange.getNorm() / (DRIVE_PERIOD / 1000.0));
        MutableTranslation2d highRobotAcceleration = new MutableTranslation2d(neededVelocityChange).
                normalize().times(maxAcceleration);

        if (additionalRobotRelativeAcceleration != null) {
            highRobotAcceleration.plus(additionalRobotRelativeAcceleration);
        }


        double currentAngleSpeed = currentSpeeds.omegaRadiansPerSecond;
        double neededAngleChange = targetAngularSpeed - currentAngleSpeed;
        double highAngularAcceleration = Math.copySign(min(accelerationLimit.angularAcceleration,
                Math.abs(targetAngularSpeed - currentAngleSpeed) / (DRIVE_PERIOD / 1000.0)), neededAngleChange);
        double lowAngularAcceleration = Math.copySign(min(
                        accelerationLimit.angularAcceleration,
                        (Math.abs(currentAngleSpeed) * DRIVE_FEEDFORWARD.kv) / DRIVE_ROTATIONAL_KA),
                -currentAngleSpeed);

        double maxUsableVoltage = getMaxUsableVoltage();

        if (isUnderMaxVoltage(getChassisSpeedsFromAcceleration(highRobotAcceleration, highAngularAcceleration, currentSpeeds),
                maxUsableVoltage)) {
            // Everything is using their highest possible voltage, so we can't go any higher.
            setCommandedVoltages(getChassisSpeedsFromAcceleration(highRobotAcceleration, highAngularAcceleration, currentSpeeds));
        } else if (isUnderMaxVoltage(
                getChassisSpeedsFromAcceleration(lowRobotAcceleration, lowAngularAcceleration, currentSpeeds),
                maxUsableVoltage)) {
            // Everything is using their lowest possible voltage, so we can't go any lower.
            setCommandedVoltages(getChassisSpeedsFromAcceleration(lowRobotAcceleration, highAngularAcceleration, currentSpeeds));
        } else {
            // Binary search for the angular acceleration that will result in the robot being under the max voltage.
            // Angular speed has priority over translational speed, so we'll assume that we're using the lowest translational
            // acceleration and try to find the highest angular acceleration that will result in the robot being under the max
            // voltage.
            double angularAcceleration;
            {
                double low = lowAngularAcceleration;
                double high = highAngularAcceleration;
                while (high - low > Constants.DRIVE_ANGULAR_BINARY_SEARCH_TOLERANCE) {
                    double mid = (low + high) / 2;
                    ChassisSpeeds chassisSpeeds = getChassisSpeedsFromAcceleration(lowRobotAcceleration, mid, currentSpeeds);
                    if (isUnderMaxVoltage(chassisSpeeds, maxUsableVoltage)) {
                        low = mid;
                    } else {
                        high = mid;
                    }
                }
                angularAcceleration = low;
            }

            // Binary search for the highest translational acceleration that will result in the robot being under the max voltage.
            // We'll use the highest angular acceleration we found above, and try to maximize the translational acceleration
            // within the allowed voltage budget.
            Translation2d translationalAcceleration;
            {
                final MutableTranslation2d low = lowRobotAcceleration;
                final MutableTranslation2d high = highRobotAcceleration;
                while (dist2(high.minus(low)) > Constants.DRIVE_TRANSLATIONAL_BINARY_SEARCH_TOLERANCE) {
                    // This is technically changing the direction of the vector. Would this cause issues?
                    low.plus(high).times(0.5);
                    ChassisSpeeds chassisSpeeds = getChassisSpeedsFromAcceleration(low, angularAcceleration, currentSpeeds);
                    if (!isUnderMaxVoltage(chassisSpeeds, maxUsableVoltage)) {
                        low.times(2).minus(high); // Undo changes to low
                        high.plus(low).times(0.5); // Set high to what low was
                    } // else, low has already been updated to be the correct value.
                }

                // Should we be doing the anti-tip acceleration here?
                double maxAccel = getMaxAccelerationToAvoidTipping(angleOf(low));
                if (low.getNorm() > maxAccel) {
                    low.normalize().times(maxAccel);
                }

                translationalAcceleration = low.getTranslation2d();
            }
            expectedAcceleration = translationalAcceleration;

            setCommandedVoltages(getChassisSpeedsFromAcceleration(translationalAcceleration, angularAcceleration, currentSpeeds));
        }
    }

    @Contract(pure = true)
    private boolean isUnderMaxVoltage(ChassisSpeeds chassisSpeeds, double maxUsableVoltage) {
        SwerveModuleState[] swerveModuleStates = SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        for (SwerveModuleState swerveModuleState : swerveModuleStates) {
            if (swerveModuleState.speedMetersPerSecond > maxUsableVoltage) { // Speed m/s is actually voltage
                return false;
            }
        }
        return false;
    }

    private double getMaxUsableVoltage() {
        return 10 - DRIVE_FEEDFORWARD.ks;
    }


    @Contract(mutates = "", pure = true)
    public @NotNull ChassisSpeeds getChassisSpeedsFromAcceleration(@NotNull Translation2d robotRelativeAcceleration,
                                                                   double angularAcceleration,
                                                                   @NotNull ChassisSpeeds currentSpeeds) {
        final double kV = DRIVE_FEEDFORWARD.kv;
        final double kA = DRIVE_FEEDFORWARD.ka;
        final double kAR = DRIVE_ROTATIONAL_KA;

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                currentSpeeds.vxMetersPerSecond * kV + robotRelativeAcceleration.getX() * kA,
                currentSpeeds.vyMetersPerSecond * kV + robotRelativeAcceleration.getY() * kA,
                currentSpeeds.omegaRadiansPerSecond * kV + angularAcceleration * kAR
        );


        SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        return chassisSpeeds;
    }


    private static final double RIGHT_BACK_ANGLE_RADIANS;
    private static final double RIGHT_FRONT_ANGLE_RADIANS;
    private static final double LEFT_FRONT_ANGLE_RADIANS;
    private static final double LEFT_BACK_LEFT_ANGLE_RADIANS;

    static {
        RIGHT_FRONT_ANGLE_RADIANS = angleOf(SWERVE_RIGHT_FRONT_LOCATION.plus(ROBOT_CENTER_OF_GRAVITY)).getRadians();
        LEFT_FRONT_ANGLE_RADIANS = angleOf(SWERVE_LEFT_FRONT_LOCATION.plus(ROBOT_CENTER_OF_GRAVITY)).getRadians();
        LEFT_BACK_LEFT_ANGLE_RADIANS = angleOf(SWERVE_LEFT_BACK_LOCATION.plus(ROBOT_CENTER_OF_GRAVITY)).getRadians();
        RIGHT_BACK_ANGLE_RADIANS = angleOf(SWERVE_RIGHT_BACK_LOCATION.plus(ROBOT_CENTER_OF_GRAVITY)).getRadians();
    }

    /**
     * This method assumes that the robot is a square.
     *
     * @param accelerationAngle The angle of the acceleration vector in radians.
     * @return The maximum translational acceleration in that direction.
     */
    @Contract(mutates = "", pure = true)
    public double getMaxAccelerationToAvoidTipping(Rotation2d accelerationAngle) {
        double accelerationAngleRadians = mod(accelerationAngle.getRadians() + Math.PI, Math.PI * 2) - Math.PI;

        double distanceCgToFrame;

        if (accelerationAngleRadians > RIGHT_BACK_ANGLE_RADIANS && accelerationAngleRadians <= LEFT_FRONT_ANGLE_RADIANS) {
            distanceCgToFrame = Math.cos(accelerationAngleRadians) *
                    (SWERVE_RIGHT_FRONT_LOCATION.getX() - ROBOT_CENTER_OF_GRAVITY.getX());
        } else if (accelerationAngleRadians > LEFT_FRONT_ANGLE_RADIANS && accelerationAngleRadians <= LEFT_BACK_LEFT_ANGLE_RADIANS) {
            distanceCgToFrame = Math.cos(accelerationAngleRadians - (Math.PI / 4)) *
                    (SWERVE_LEFT_FRONT_LOCATION.getY() - ROBOT_CENTER_OF_GRAVITY.getY());
        } else if (accelerationAngleRadians > LEFT_BACK_LEFT_ANGLE_RADIANS && accelerationAngleRadians <= RIGHT_FRONT_ANGLE_RADIANS) {
            distanceCgToFrame = Math.cos(accelerationAngleRadians - (Math.PI / 2)) *
                    (SWERVE_LEFT_BACK_LOCATION.getX() - ROBOT_CENTER_OF_GRAVITY.getX());
        } else {
            distanceCgToFrame = Math.cos(accelerationAngleRadians - (3 * (Math.PI / 4))) *
                    (SWERVE_RIGHT_BACK_LOCATION.getY() - ROBOT_CENTER_OF_GRAVITY.getY());
        }

        return distanceCgToFrame * 10; // TODO: Tune this
    }


    public void setCommandedVoltages(ChassisSpeeds commandVoltage) {
        SmartDashboard.putNumber("Drive Command X Voltage", commandVoltage.vxMetersPerSecond);
        SmartDashboard.putNumber("Drive Command Y Voltage", commandVoltage.vyMetersPerSecond);
        SmartDashboard.putNumber("Drive Command Rotational Voltage", commandVoltage.omegaRadiansPerSecond);

        SwerveModuleState[] moduleStates = SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(commandVoltage);

        boolean rotate = commandVoltage.vxMetersPerSecond != 0 ||
                commandVoltage.vyMetersPerSecond != 0 ||
                commandVoltage.omegaRadiansPerSecond != 0;

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, getMaxUsableVoltage()); // In some rare cases, we can still
        // command above the max voltage. Ex: if the available voltage drops faster than we can decelerate.
        setSwerveModuleStates(moduleStates, rotate);
    }

    public void setSwerveModuleStates(SwerveModuleState[] moduleStates, boolean rotate) {
        for (int i = 0; i < 4; i++) {
            double currentAngle = getWheelRotation(i);
            SwerveModuleState targetState = SwerveModuleState.optimize(moduleStates[i], Rotation2d.fromDegrees(currentAngle));

            double targetAngle = mod(targetState.angle.getDegrees(), 360);

            double angleDiff = getAngleDiff(targetAngle, currentAngle);

            if (Math.abs(angleDiff) > 0.1 && rotate) { // Only  update the setpoint if we're not already there and moving
                setSwerveMotorPosition(i, getRelativeSwervePosition(i) + angleDiff);
            }

            setMotorVoltage(i, targetState.speedMetersPerSecond);

            SmartDashboard.putNumber("Swerve Motor " + i + " Target Position", getRelativeSwervePosition(i) + angleDiff);
            SmartDashboard.putNumber("Swerve Motor " + i + " Error", angleDiff);
        }
    }

    /**
     * Sets the motor voltage
     *
     * @param module        The module to set the voltage on
     * @param outputVoltage The target outputVoltage
     */
    public void setMotorVoltage(int module, double outputVoltage) {
        if (module < 0 || module > 3) {
            throw new IllegalArgumentException("Module must be between 0 and 3");
        }

        swerveDriveMotors[module].set(ControlMode.PercentOutput,
                (outputVoltage / Constants.SWERVE_DRIVE_VOLTAGE_LIMIT) + DRIVE_FEEDFORWARD.ks);
        SmartDashboard.putNumber("Out Volts " + module, outputVoltage);
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

    @NotNull Translation2d lastRequestedVelocity = new Translation2d();
    double lastRequestedRotation = 0;

    private double lastLoopTime = 0;

    /**
     * Converts a field relative velocity to be robot relative
     *
     * @param fieldRelativeVelocity a field relative velocity
     * @return a robot relative velocity
     */
    private Translation2d toRobotRelative(Translation2d fieldRelativeVelocity) {
        return fieldRelativeVelocity.rotateBy(RobotTracker.getInstance().getGyroAngle().unaryMinus());
    }

    /**
     * Converts a robot relative velocity to be field relative
     *
     * @param robotRelativeTranslation a field relative velocity
     * @return a field relative velocity
     */
    private Translation2d toFieldRelative(Translation2d robotRelativeTranslation) {
        return robotRelativeTranslation.rotateBy(RobotTracker.getInstance().getGyroAngle());
    }

    private final double[] lastWheelSpeeds = new double[4];
    private final double[] lastWheelSpeedsTime = new double[4];

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
        setAutoRotation(Rotation2d.fromDegrees(angle));
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

    private final ReentrantLock swerveAutoControllerLock = new ReentrantLock();
    private @Nullable HolonomicDriveController swerveAutoController;
    boolean swerveAutoControllerInitialized = false;

    public void setAutoPath(Trajectory trajectory) {
        currentAutoTrajectoryLock.lock();
        try {
            swerveAutoControllerInitialized = false;
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
    private volatile @NotNull TrapezoidProfile.State autoAimingRotationGoal = new TrapezoidProfile.State(0, 0);

    public void setAutoAiming(boolean autoAiming) {
        isAutoAiming = autoAiming;
    }

    public void setAutoAiming(TrapezoidProfile.State autoAimingRotationGoal) {
        isAutoAiming = true;
        this.autoAimingRotationGoal = autoAimingRotationGoal;
    }


    private double nextAllowedPrintError = 0;

    @SuppressWarnings("ProhibitedExceptionCaught")
    private void updateRamsete() {
        currentAutoTrajectoryLock.lock();
        try {
            if (!swerveAutoControllerInitialized) {
                resetAuto();
                assert currentAutoTrajectory != null;
                swerveAutoControllerInitialized = true;
            }

            Trajectory.State goal = currentAutoTrajectory.sample(Timer.getFPGATimestamp() - autoStartTime);

            Rotation2d targetHeading = autoTargetHeading;


            try {
                if (swerveAutoController == null) {
                    DriverStation.reportError("swerveAutoController is null",
                            Thread.getAllStackTraces().get(Thread.currentThread()));
                    resetAuto();
                }
                ChassisSpeeds adjustedSpeeds = swerveAutoController.calculate(
                        RobotTracker.getInstance().getRawPose(),
                        goal,
                        targetHeading);

                if (isAutoAiming) {
                    adjustedSpeeds.omegaRadiansPerSecond = getTurnPidDeltaSpeed(autoAimingRotationGoal);
                }

                swerveDrive(new Translation2d(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond),
                        adjustedSpeeds.omegaRadiansPerSecond,
                        new Translation2d(goal.accelerationMetersPerSecondSq, 0)
                                .rotateBy(new Rotation2d(goal.curvatureRadPerMeter))); // Is this correct?
                if (swerveAutoController.atReference() && (Timer.getFPGATimestamp() - autoStartTime) >= currentAutoTrajectory.getTotalTimeSeconds()) {
                    setDriveState(DriveState.DONE);
                    stopMovement();
                }
            } catch (NullPointerException exception) {
                if (Timer.getFPGATimestamp() > nextAllowedPrintError) {
                    exception.printStackTrace();
                    nextAllowedPrintError = Timer.getFPGATimestamp() + 2;
                }
            }
        } finally {
            currentAutoTrajectoryLock.unlock();
        }
    }

    /**
     * @param autoAimingRotationGoal The goal to aim at (in radians)
     * @return The speed to turn at (in radians/s)
     */
    private double getTurnPidDeltaSpeed(@NotNull TrapezoidProfile.State autoAimingRotationGoal) {
        turnPID.setSetpoint(autoAimingRotationGoal.position);

        if (Timer.getFPGATimestamp() - 0.2 > lastTurnUpdate) {
            turnPID.reset();
        } else if (turnPID.getPositionError() > Math.toRadians(7)) {
            // This is basically an I-Zone
            turnPID.resetI();
        }
        lastTurnUpdate = Timer.getFPGATimestamp();
        return Math.max(
                turnPID.calculate(RobotTracker.getInstance().getGyroAngle().getRadians()) + autoAimingRotationGoal.velocity,
                Constants.TURN_SPEED_LIMIT_WHILE_AIMING);
    }

    public void setAutoRotation(@NotNull Rotation2d rotation) {
        currentAutoTrajectoryLock.lock();
        try {
            autoTargetHeading = rotation;
        } finally {
            currentAutoTrajectoryLock.unlock();
        }
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
                swerveDrive(new Translation2d(0, 0), 0, null);
        }
    }

    synchronized public boolean isAiming() {
        return isAiming;
    }

    public void setRotation(Rotation2d angle) {
        wantedHeading = angle;
        driveState = DriveState.TURN;
        rotateAuto = true;
        isAiming = !isTurningDone();
    }

    public void setRotation(double angle) {
        setRotation(Rotation2d.fromDegrees(angle));
    }


    public boolean isTurningDone() {
        Rotation2d currentHeading = RobotTracker.getInstance().getGyroAngle();
        synchronized (this) {
            double error = wantedHeading.minus(currentHeading).getDegrees();
            return (Math.abs(error) < Constants.MAX_TURN_ERROR);
        }
    }

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


        double pidDeltaSpeed = getTurnPidDeltaSpeed(goal);
        double curSpeed = RobotTracker.getInstance().getLatencyCompedChassisSpeeds().omegaRadiansPerSecond;

        Translation2d targetVelocity = new Translation2d(controllerDriveInputs.getX(), controllerDriveInputs.getY())
                .times(DRIVE_HIGH_SPEED_M);
        if (useFieldRelative) {
            swerveDriveFieldRelative(targetVelocity, pidDeltaSpeed, null);
        } else {
            swerveDrive(targetVelocity, pidDeltaSpeed, null);
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
        }

        logData("Turn Position Error", Math.toDegrees(turnPID.getPositionError()));
        logData("Turn Actual Speed", curSpeed);
        logData("Turn PID Command", pidDeltaSpeed);
        logData("Turn PID Setpoint Position", goal.position);
        logData("Turn PID Setpoint Velocity", goal.velocity);
        logData("Turn PID Measurement", RobotTracker.getInstance().getGyroAngle().getRadians());
    }

    public void stopMovement() {
        setDriveState(DriveState.STOP);
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
            return mod(getRelativeSwervePosition(moduleNumber), 360);
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

    public Translation2d getExpectedAcceleration() {
        return expectedAcceleration;
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
            Thread.sleep(20);
        }
    }
}
