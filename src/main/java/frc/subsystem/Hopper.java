package frc.subsystem;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.subsystem.Intake.IntakeSolState;
import frc.subsystem.Intake.IntakeState;
import frc.utility.Limelight;
import frc.utility.OrangeUtility;
import frc.utility.Timer;
import frc.utility.controllers.LazyCANSparkMax;
import org.jetbrains.annotations.NotNull;

import static frc.robot.Constants.HOPPER_CURRENT_LIMIT;

public final class Hopper extends AbstractSubsystem {

    private static @NotNull Hopper INSTANCE = new Hopper();
    private final @NotNull LazyCANSparkMax hopperMotor;
    private RelativeEncoder outtakeWheelsQuadrature;
    private LazyCANSparkMax outtakeWheels;
    private double lastDetectionTime;
    private final Limelight intakeLimelight = Limelight.getInstance(Constants.intakeLimelightName);

    public static Hopper getInstance() {
        return INSTANCE;
    }

    public enum HopperState {
        ON, OFF, REVERSE, SLOW
    }

    HopperState wantedHopperState = HopperState.OFF;

    /**
     * Outtake can either be OFF or INTAKE, or EJECT
     */
    public enum OuttakeState {
        OFF,
        INTAKE,
        EJECT,
    }

    private OuttakeState outtakeState = OuttakeState.OFF;

    public enum BallColor {
        RED,
        BLUE,
        NO_BALL
    }

    BallColor opposingAllianceColor = BallColor.BLUE;

    private Hopper() {
        super(10, 5);
        hopperMotor = new LazyCANSparkMax(Constants.HOPPER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        hopperMotor.setSmartCurrentLimit(HOPPER_CURRENT_LIMIT);

        hopperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        hopperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        hopperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        hopperMotor.setControlFramePeriodMs(25);

        outtakeWheels = new LazyCANSparkMax(Constants.OUTTAKE_CAN_ID, MotorType.kBrushless);
        outtakeWheels.setIdleMode(IdleMode.kCoast);
        outtakeWheels.setInverted(true);
        outtakeWheels.setSmartCurrentLimit(Constants.OUTTAKE_CURRENT_LIMIT);
        outtakeWheels.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        outtakeWheels.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
        outtakeWheels.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 25);
        outtakeWheels.setControlFramePeriodMs(25);

        outtakeWheelsQuadrature = outtakeWheels.getEncoder();
        outtakeWheelsQuadrature.setPositionConversionFactor(Constants.OUTTAKE_REDUCTION);
        outtakeWheelsQuadrature.setMeasurementPeriod(Constants.OUTTAKE_MEASUREMENT_PERIOD_MS);

        outtakeWheels.disableVoltageCompensation();
        outtakeWheels.burnFlash();
    }

    /**
     * Uses Sendable Chooser to decide Alliance Color
     */
    private void updateAllianceColor() {
        if (Robot.sideChooser.getSelected().equals(Robot.BLUE)) {
            opposingAllianceColor = BallColor.RED;
        } else {
            opposingAllianceColor = BallColor.BLUE;
        }
    }

    /**
     * Updates state of outtake based on color sensor and intake direction
     */
    private void updateOuttakeState() {
        // If color sensor detects a ball or
        Intake intake = Intake.getInstance();
        if (intake.wantedIntakeState == IntakeState.INTAKE && intake.getIntakeSolState() == IntakeSolState.OPEN) {
            if (getBallColor() == opposingAllianceColor) {
                lastDetectionTime = Timer.getFPGATimestamp();
                outtakeState = OuttakeState.EJECT;
            } else if (Timer.getFPGATimestamp() - lastDetectionTime < Constants.OUTTAKE_RUN_PERIOD) {
                outtakeState = OuttakeState.EJECT;
            } else {
                outtakeState = OuttakeState.OFF;
            }
        } else {
            outtakeState = OuttakeState.OFF;
        }
    }

    /**
     * Sets the percent outtake between 1 and -1
     */
    private void setOuttakePercentOutput(double percentOutput) {
        outtakeWheels.set(percentOutput);
        //outtakeWheels.getPIDController().setReference(9 * percentOutput, ControlType.kVoltage);
    }

    public OuttakeState getOuttakeState() {
        return outtakeState;
    }

    public BallColor getBallColor() {
        @NotNull BallColor currentBallColor;

        if (intakeLimelight.isTargetVisible()) {
            currentBallColor = opposingAllianceColor;
        } else {
            currentBallColor = BallColor.NO_BALL;
        }

        return currentBallColor;
    }

    private void setHopperStatePrivate(HopperState hopperState) {
        // Forces Hopper to run in reverse if outtake is ejecting
        if (getOuttakeState() == OuttakeState.EJECT) {
            hopperState = HopperState.OFF;
        }

        switch (hopperState) {
            case ON:
                hopperMotor.set(Constants.HOPPER_SPEED);
                Shooter.getInstance().runFeederWheelReversed = true;
                break;
            case OFF:
                hopperMotor.set(0);
                Shooter.getInstance().runFeederWheelReversed = false;
                break;
            case REVERSE:
                hopperMotor.set(-Constants.HOPPER_SPEED);
                Shooter.getInstance().runFeederWheelReversed = false;
                break;
            case SLOW:
                hopperMotor.set(Constants.HOPPER_SPEED / 2);
                Shooter.getInstance().runFeederWheelReversed = false;
                break;
        }
    }

    @Override
    public void update() {
        updateAllianceColor();
        getBallColor();
        updateOuttakeState();

        switch (outtakeState) {
            case OFF:
                setOuttakePercentOutput(0);
                break;
            case EJECT:
                setOuttakePercentOutput(Constants.OUTTAKE_SPEED_FACTOR);
                break;
            case INTAKE:
                setOuttakePercentOutput(-Constants.OUTTAKE_SPEED_FACTOR);
                break;
        }

        setHopperStatePrivate(wantedHopperState);
    }

    public void setHopperState(HopperState hopperState) {
        wantedHopperState = hopperState;
    }

    @Override
    public void selfTest() {
        setHopperState(HopperState.ON);
        OrangeUtility.sleep(5000);
        setHopperState(HopperState.OFF);
        OrangeUtility.sleep(5000);

        setOuttakePercentOutput(Constants.OUTTAKE_SPEED_FACTOR);
        System.out.println("Ejecting");
        OrangeUtility.sleep(Constants.TEST_TIME_MS);

        setOuttakePercentOutput(-Constants.OUTTAKE_SPEED_FACTOR);
        System.out.println("Intaking");
        OrangeUtility.sleep(Constants.TEST_TIME_MS);

        System.out.println("Test Finished");
        setOuttakePercentOutput(0);
    }

    @Override
    public void logData() {
        logData("Hopper Motor Current", hopperMotor.getOutputCurrent());

        logData("Outtae State", outtakeState);
        logData("Outtake SetVoltage", outtakeWheels.getSetpoint());
        logData("Outtake Velocity", outtakeWheelsQuadrature.getVelocity());
        logData("Current Ball Color", getBallColor());
    }

    @Override
    public void close() throws Exception {
        setHopperState(HopperState.OFF);
        INSTANCE = new Hopper();
    }
}
