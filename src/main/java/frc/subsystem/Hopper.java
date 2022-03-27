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
    private boolean disableEject = false;
    private final Limelight intakeLimelight = Limelight.getInstance(Constants.INTAKE_LIMELIGHT_NAME);

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
    }

    /**
     * Uses Sendable Chooser to decide Alliance Color
     */
    private void updateAllianceColor() {
        if (Robot.sideChooser.getSelected().equals(Robot.BLUE)) {
            opposingAllianceColor = BallColor.RED;
            friendlyAllianceColor = BallColor.BLUE;

            if (Constants.OUTTAKE_ALWAYS_INTAKE) {
                // Will detect opposite color balls and outtake when it sees them
                intakeLimelight.setPipeline(1);
            } else {
                // Will detect same colored balls and intake when it sees them
                intakeLimelight.setPipeline(0);
            }
        } else {
            opposingAllianceColor = BallColor.BLUE;
            friendlyAllianceColor = BallColor.RED;
            if (Constants.OUTTAKE_ALWAYS_INTAKE) {
                // Will detect same colored balls and intake when it sees them
                intakeLimelight.setPipeline(0);
            } else {
                // Will detect opposite color balls and outtake when it sees them
                intakeLimelight.setPipeline(1);
            }
        }
    }

    /**
     * Toggles disableEject
     */
    public void toggleEjectOverride() {
        disableEject = !disableEject;
    }

    /**
     * Updates state of outtake based on color sensor and intake direction
     */
    private void updateOuttakeState() {
        // If color sensor detects a ball or
        Intake intake = Intake.getInstance();

        // Override for all other possible states that reverses outtake if hopper is in reverse
        if (wantedHopperState == HopperState.REVERSE && intake.getIntakeSolState() == IntakeSolState.OPEN) {
            outtakeState = OuttakeState.EJECT;
            return;
        }

        if (Constants.OUTTAKE_ALWAYS_INTAKE) {
            // Intake is running and open
            if (wantedHopperState == HopperState.ON && intake.getIntakeSolState() == IntakeSolState.OPEN) {
                // Ball Color is opposite
                if (getBallColor() == opposingAllianceColor) {
                    lastDetectionTime = Timer.getFPGATimestamp();
                    outtakeState = OuttakeState.EJECT;
                    // Opposite ball color detected within a certain time frame
                } else if (Timer.getFPGATimestamp() - lastDetectionTime < Constants.OUTTAKE_RUN_PERIOD) {
                    outtakeState = OuttakeState.EJECT;
                } else {
                    outtakeState = OuttakeState.INTAKE;
                }
            } else {
                outtakeState = OuttakeState.OFF;
            }
        } else {
            // Intake is running and open
            if (wantedHopperState == HopperState.ON && intake.getIntakeSolState() == IntakeSolState.OPEN) {
                // Ball Color is opposite
                if (getBallColor() == friendlyAllianceColor) {
                    lastDetectionTime = Timer.getFPGATimestamp();
                    outtakeState = OuttakeState.INTAKE;
                    // Opposite ball color detected within a certain time frame
                } else if (Timer.getFPGATimestamp() - lastDetectionTime < Constants.OUTTAKE_RUN_PERIOD) {
                    outtakeState = OuttakeState.INTAKE;
                } else {
                    outtakeState = OuttakeState.EJECT;
                }
            } else {
                outtakeState = OuttakeState.OFF;
            }
        }
    }

    private void updateOuttakeStateOverridden() {
        Intake intake = Intake.getInstance();
        if (wantedHopperState == HopperState.REVERSE) {
            outtakeState = OuttakeState.EJECT;
        } else if (intake.wantedIntakeState == IntakeState.INTAKE && intake.getIntakeSolState() == IntakeSolState.OPEN) {
            outtakeState = OuttakeState.INTAKE;
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

        if (Constants.OUTTAKE_ALWAYS_INTAKE) {
            if (intakeLimelight.getVerticalOffset() < Constants.OUTTAKE_VERTICAL_OFFSET_THRESHOLD) {
                currentBallColor = opposingAllianceColor;
            } else {
                currentBallColor = BallColor.NO_BALL;
            }
        } else {
            if (intakeLimelight.getVerticalOffset() < Constants.OUTTAKE_VERTICAL_OFFSET_THRESHOLD) {
                currentBallColor = friendlyAllianceColor;
            } else {
                currentBallColor = BallColor.NO_BALL;
            }
        }

        return currentBallColor;
    }

    @Override
    public void update() {
        updateAllianceColor();
        getBallColor();

        if (!disableEject) {
            updateOuttakeState();
        } else {
            updateOuttakeStateOverridden();
        }

        // Outtake motor control

        switch (outtakeState) {
            case OFF:
                setOuttakePercentOutput(0);
                break;
            case EJECT:
                setOuttakePercentOutput(Constants.EJECT_OUTTAKE_SPEED);
                break;
            case INTAKE:
                setOuttakePercentOutput(-Constants.INTAKEING_OUTTAKE_SPEED);
                break;
        }


        // Hopper Motor Control
        switch (wantedHopperState) {
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

    public void setHopperState(HopperState hopperState) {
        wantedHopperState = hopperState;
    }

    @Override
    public void selfTest() {
        setHopperState(HopperState.ON);
        OrangeUtility.sleep(5000);
        setHopperState(HopperState.OFF);
        OrangeUtility.sleep(5000);

        setOuttakePercentOutput(Constants.INTAKEING_OUTTAKE_SPEED);
        System.out.println("Ejecting");
        OrangeUtility.sleep(Constants.TEST_TIME_MS);

        setOuttakePercentOutput(-Constants.INTAKEING_OUTTAKE_SPEED);
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
    }

    @Override
    public void close() throws Exception {
        setHopperState(HopperState.OFF);
        INSTANCE = new Hopper();
    }
}
