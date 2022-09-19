package frc.subsystem;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.subsystem.Intake.IntakeSolState;
import frc.subsystem.Shooter.FeederWheelState;
import frc.subsystem.Shooter.ShooterState;
import frc.utility.Limelight;
import frc.utility.OrangeUtility;
import frc.utility.controllers.LazyCANSparkMax;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

import java.util.Objects;

import static frc.robot.Constants.*;

public final class Hopper extends AbstractSubsystem {

    private static @NotNull Hopper INSTANCE = new Hopper();
    private final @NotNull LazyCANSparkMax hopperMotor;
    private final RelativeEncoder outtakeWheelsQuadrature;
    private final LazyCANSparkMax outtakeWheels;
    private double lastDetectionTime;
    private boolean disableEject = true;
    private final Limelight intakeLimelight = Limelight.getInstance(Constants.INTAKE_LIMELIGHT_NAME);
    private boolean isBeamBreakEnabled = true;

    private final DigitalInput beamBreak;
    private double lastBeamBreakOpenTime = 0;

    public static Hopper getInstance() {
        return INSTANCE;
    }

    public void resetBeamBreakOpenTime() {
        lastBeamBreakOpenTime = Timer.getFPGATimestamp();
    }

    public enum HopperState {
        ON, OFF, REVERSE, SLOW
    }

    HopperState wantedHopperState = HopperState.OFF;

    /**
     * Outtake can either be OFF or INTAKE, or AUTO_EJECT
     */
    public enum OuttakeState {
        OFF,
        INTAKE,
        AUTO_EJECT,
        MANUAL_EJECT
    }

    private OuttakeState outtakeState = OuttakeState.OFF;

    public enum BallColor {
        RED,
        BLUE,
        NO_BALL
    }

    BallColor opposingAllianceColor = BallColor.BLUE;
    BallColor friendlyAllianceColor = BallColor.RED;

    private Hopper() {
        super(Constants.HOPPER_PERIOD, 5);
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

        beamBreak = new DigitalInput(Constants.BEAM_BREAK_DIO_ID);
    }

    /**
     * Uses Sendable Chooser to decide Alliance Color
     */
    private void updateAllianceColor() {
        if (Objects.equals(Robot.sideChooser.getSelected(), "blue")) {
            opposingAllianceColor = BallColor.RED;
            friendlyAllianceColor = BallColor.BLUE;

            // Will detect opposite color balls and outtake when it sees them
            intakeLimelight.setPipeline(1);
        } else {
            opposingAllianceColor = BallColor.BLUE;
            friendlyAllianceColor = BallColor.RED;

            // Will detect same colored balls and intake when it sees them
            intakeLimelight.setPipeline(0);
        }
    }

    /**
     * Toggles disableEject
     */
    public void toggleEjectOverride() {
        disableEject = !disableEject;
    }

    /**
     * Toggles disableEject
     */
    public void setEjectOverride(boolean ejectOverride) {
        disableEject = ejectOverride;
    }

    /**
     * Updates state of outtake based on color sensor and intake direction
     */
    private void updateOuttakeState() {
        Intake intake = Intake.getInstance();

        if (wantedHopperState == HopperState.REVERSE) {
            outtakeState = intake.getIntakeSolState() == IntakeSolState.OPEN ? OuttakeState.MANUAL_EJECT : OuttakeState.OFF;
            return;
        }
        if (wantedHopperState == HopperState.OFF) {
            outtakeState = OuttakeState.OFF;
            return;
        }

        if (getBallColor() == opposingAllianceColor
                && intake.getIntakeSolState() == IntakeSolState.OPEN
                && !disableEject) { // If disable eject is on, it will not outtake
            lastDetectionTime = Timer.getFPGATimestamp();
            outtakeState = OuttakeState.AUTO_EJECT;
        } else if (Timer.getFPGATimestamp() - lastDetectionTime < Constants.OUTTAKE_RUN_PERIOD
                && intake.getIntakeSolState() == IntakeSolState.OPEN) {
            // Ensure that we keep outtaking for a minimum amount of time
            outtakeState = OuttakeState.AUTO_EJECT;
        } else {
            outtakeState = OuttakeState.INTAKE;
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

    @Contract(pure = true)
    public BallColor getBallColor() {
        @NotNull BallColor currentBallColor;

        if (intakeLimelight.getVerticalOffset() < Constants.OUTTAKE_VERTICAL_OFFSET_THRESHOLD) {
            currentBallColor = opposingAllianceColor;
        } else {
            currentBallColor = BallColor.NO_BALL;
        }

        return currentBallColor;
    }


    public double getLastBeamBreakOpenTime() {
        return lastBeamBreakOpenTime;
    }

    @Override
    public void update() {
        updateAllianceColor();
        updateOuttakeState();

        if (!isBeamBroken()) {
            lastBeamBreakOpenTime = Timer.getFPGATimestamp();
        }

        // Outtake motor control

        switch (outtakeState) {
            case OFF:
                setOuttakePercentOutput(0);
                break;
            case AUTO_EJECT:
                setOuttakePercentOutput(Constants.OUTTAKE_AUTO_EJECTION_SPEED);
                break;
            case INTAKE:
                setOuttakePercentOutput(Constants.OUTTAKE_SPEED);
                break;
            case MANUAL_EJECT:
                setOuttakePercentOutput(Constants.OUTTAKE_MANUAL_EJECTION_SPEED);
                break;
        }


        // Hopper Motor Control
        switch (wantedHopperState) {
            case ON:
                if (outtakeState == OuttakeState.AUTO_EJECT
                        && Shooter.getInstance().getFeederWheelState() != FeederWheelState.FORWARD) {
                    setHopperSpeed(HOPPER_OUTTAKING_SPEED);
                } else {
                    setHopperSpeed(HOPPER_SPEED);
                }
                break;
            case OFF:
                if (isBeamBroken() &&
                        !(Shooter.getInstance().getFeederWheelState() == FeederWheelState.FORWARD &&
                                Shooter.getInstance().getShooterState() == ShooterState.ON)) {
                    //If a ball is blocking the beam break sensor we want to run the hopper to move the ball up to unblock it.
                    setHopperSpeed(HOPPER_SPEED);
                } else {
                    setHopperSpeed(0);
                }
                break;
            case REVERSE:
                setHopperSpeed(-HOPPER_SPEED);
                break;
            case SLOW:
                setHopperSpeed(HOPPER_SLOW_SPEED);
                break;
        }
    }

    private void setHopperSpeed(double speed) {
        synchronized (hopperMotor) {
            hopperMotor.set(speed);
        }
    }

    public boolean isHopperMotorRunning() {
        synchronized (hopperMotor) {
            return hopperMotor.get() != 0;
        }
    }

    /**
     * @return True if the beam break is broken. (ie a ball is in the way)
     */
    public boolean isBeamBroken() {
        if (IS_PRACTICE || !isBeamBreakEnabled) return false;
        return !beamBreak.get();
    }


    public boolean isBeamBreakEnabled() {
        return isBeamBreakEnabled;
    }

    public void setBeamBreakEnabled(boolean isBeamBreakEnabled) {
        this.isBeamBreakEnabled = isBeamBreakEnabled;
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

        setOuttakePercentOutput(-Constants.INTAKE_EJECTION_SPEED);
        System.out.println("Ejecting");
        OrangeUtility.sleep(Constants.TEST_TIME_MS);

        setOuttakePercentOutput(Constants.INTAKE_EJECTION_SPEED);
        System.out.println("Intaking");
        OrangeUtility.sleep(Constants.TEST_TIME_MS);

        System.out.println("Test Finished");
        setOuttakePercentOutput(0);
    }

    @Override
    public void logData() {
        logData("Hopper Motor Current", hopperMotor.getOutputCurrent());
        logData("Outtake State", outtakeState);
        logData("Outtake SetVoltage", outtakeWheels.getSetpoint());
        logData("Outtake Velocity", outtakeWheelsQuadrature.getVelocity());
        logData("Outtake Current", outtakeWheels.getOutputCurrent());
        logData("Current Ball Color", getBallColor());
        logData("Eject Disabled", disableEject);
        logData("Hopper State", wantedHopperState);
        logData("Is Beam Broken", isBeamBroken());
        logData("Last Beam Break Open Time", getLastBeamBreakOpenTime());
        logData("Eject Motor Temperature", outtakeWheels.getMotorTemperature());
        logData("Hopper Motor Temperature", hopperMotor.getMotorTemperature());
    }

    @Override
    public void close() throws Exception {
        setHopperState(HopperState.OFF);
        INSTANCE = new Hopper();
    }
}
