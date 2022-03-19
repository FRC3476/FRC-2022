package frc.subsystem;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.subsystem.Hopper.ColorSensorStatus;
import frc.subsystem.Intake.IntakeState;
import frc.utility.OrangeUtility;
import frc.utility.Timer;
import frc.utility.controllers.LazyCANSparkMax;
import org.jetbrains.annotations.NotNull;

public class Outtake extends AbstractSubsystem {

    private Hopper hopper = Hopper.getInstance();
    private Intake intake = Intake.getInstance();

    private LazyCANSparkMax outtakeWheels;
    private RelativeEncoder outtakeWheelsQuadrature;

    private double lastDetectionTime;

    private ColorSensorStatus allianceColor = ColorSensorStatus.RED;

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
        super(Constants.OUTTAKE_PERIOD);

        updateAllianceColor();

        outtakeWheels = new LazyCANSparkMax(Constants.OUTTAKE_CAN_ID, MotorType.kBrushless);
        outtakeWheels.setIdleMode(IdleMode.kCoast);
        outtakeWheels.setInverted(false);
        outtakeWheels.setSmartCurrentLimit(Constants.OUTTAKE_CURRENT_LIMIT);
        outtakeWheels.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        outtakeWheels.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
        outtakeWheels.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 25);
        outtakeWheels.setControlFramePeriodMs(25);

        outtakeWheelsQuadrature = outtakeWheels.getEncoder();
        outtakeWheelsQuadrature.setPositionConversionFactor(Constants.OUTTAKE_REDUCTION);
        outtakeWheelsQuadrature.setMeasurementPeriod(Constants.OUTTAKE_MEASUREMENT_PERIOD_MS);

        outtakeWheels.burnFlash();
    }

    /**
     * Uses Sendable Chooser to decide Alliance Color
     */
    private void updateAllianceColor() {
        if (Robot.sideChooser.getSelected().equals(Robot.BLUE)) {
            allianceColor = ColorSensorStatus.BLUE;
        } else {
            allianceColor = ColorSensorStatus.RED;
        }
    }

    /**
     * Updates state of outtake based on color sensor and intake direction
     */
    private void updateOuttakeState() {
        // If color sensor detects a ball or
        if (hopper.getColorSensorStatus() == allianceColor || Timer.getFPGATimestamp() - lastDetectionTime < Constants.OUTTAKE_RUN_PERIOD) {
            lastDetectionTime = Timer.getFPGATimestamp();
            outtakeState = OuttakeState.EJECT;
        } else if (intake.wantedIntakeState == IntakeState.INTAKE) {
            outtakeState = OuttakeState.INTAKE;
        } else {
            outtakeState = OuttakeState.OFF;
        }
    }

    /**
     * Sets the percent outtake between 1 and -1
     */
    private void setOuttakePercentOutput(double percentOutput) {
        outtakeWheels.setVoltage(12d * percentOutput);
    }

    @Override
    public void update() {
        updateOuttakeState();

        switch (outtakeState) {
            case OFF:
                setOuttakePercentOutput(0);
                break;
            case EJECT:
                setOuttakePercentOutput(1 * Constants.OUTTAKE_SPEED_FACTOR);
                break;
            case INTAKE:
                setOuttakePercentOutput(-1 * Constants.OUTTAKE_SPEED_FACTOR);
                break;
        }
    }

    @Override
    public void selfTest() {

        setOuttakePercentOutput(1 * Constants.OUTTAKE_SPEED_FACTOR);
        System.out.println("Ejecting");
        OrangeUtility.sleep(Constants.TEST_TIME_MS);

        setOuttakePercentOutput(-1 * Constants.OUTTAKE_SPEED_FACTOR);
        System.out.println("Intaking");
        OrangeUtility.sleep(Constants.TEST_TIME_MS);

        System.out.println("Test Finished");
        setOuttakePercentOutput(0);
    }

    @Override
    public void logData() {
        logData("Outtake State", outtakeState);
        logData("Outtake Velocity", outtakeWheelsQuadrature.getVelocity());
        logData("Outtake SetVoltage", outtakeWheels.getSetVoltage());
    }

    @Override
    public void close() throws Exception {

    }
}
