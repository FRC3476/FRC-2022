package frc.subsystem;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.subsystem.Hopper.ColorSensorStatus;
import frc.subsystem.Intake.IntakeState;
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
        outtakeWheelsQuadrature.setMeasurementPeriod(Constants.OUTTAKE_MEASUREMENT_PERIOD);

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

    @Override
    public void update() {
        updateOuttakeState();

        switch (outtakeState) {
            case OFF:
                break;
            case EJECT:
                break;
            case INTAKE:
                break;
        }
    }

    @Override
    public void selfTest() {

    }

    @Override
    public void logData() {
        logData("Outtake State", outtakeState);
        logData("Outtake Velocity", outtakeWheelsQuadrature.getVelocity());
    }

    @Override
    public void close() throws Exception {

    }
}
