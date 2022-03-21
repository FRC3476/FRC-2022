package frc.subsystem;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.subsystem.Hopper.ColorSensorStatus;
import frc.subsystem.Intake.IntakeSolState;
import frc.subsystem.Intake.IntakeState;
import frc.utility.OrangeUtility;
import frc.utility.Timer;
import frc.utility.controllers.LazyCANSparkMax;
import org.jetbrains.annotations.NotNull;

public class Outtake extends AbstractSubsystem {

    //private Hopper hopper = Hopper.getInstance();
    //private Intake intake = Intake.getInstance();

    private LazyCANSparkMax outtakeWheels;

    private double lastDetectionTime;

    private RelativeEncoder outtakeWheelsQuadrature;

    private @NotNull ColorSensorStatus allianceColor = ColorSensorStatus.RED;

    private @NotNull ColorSensorStatus opposingAllianceColor = ColorSensorStatus.BLUE;

    private static @NotNull Outtake instance = new Outtake();

    public static Outtake getInstance() {
        return instance;
    }

    /**
     * Outtake can either be OFF or ON
     */
    public enum OuttakeState {
        OFF,
        INTAKE,
        EJECT,
    }

    private OuttakeState outtakeState = OuttakeState.OFF;

    private Outtake() {
        super(10);

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


        //outtakeWheels.enableVoltageCompensation(9);
        outtakeWheels.disableVoltageCompensation();
        outtakeWheels.burnFlash();
    }

    NetworkTableEntry sideEntry = NetworkTableInstance.getDefault().getEntry("FMSInfo.IsRedAlliance");

    /**
     * Uses Sendable Chooser to decide Alliance Color
     */
    private void updateAllianceColor() {
        if (Robot.sideChooser.getSelected().equals(Robot.BLUE)) {
            allianceColor = ColorSensorStatus.BLUE;
            opposingAllianceColor = ColorSensorStatus.RED;
        } else {
            allianceColor = ColorSensorStatus.RED;
            opposingAllianceColor = ColorSensorStatus.BLUE;
        }
    }

    /**
     * Updates state of outtake based on color sensor and intake direction
     */
    private void updateOuttakeState() {
        // If color sensor detects a ball or
        Hopper hopper = Hopper.getInstance();
        Intake intake = Intake.getInstance();
        if (intake.wantedIntakeState == IntakeState.INTAKE && intake.getIntakeSolState() == IntakeSolState.OPEN) {
            if (hopper.getColorSensorStatus() == opposingAllianceColor) {
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

    @Override
    public void update() {
        updateAllianceColor();
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
    }

    @Override
    public void selfTest() {

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
        logData("Outtak" +
                "e State", outtakeState);
        logData("Outtake SetVoltage", outtakeWheels.getSetpoint());
        logData("Outtake Velocity", outtakeWheelsQuadrature.getVelocity());
    }

    @Override
    public void close() throws Exception {

    }
}
