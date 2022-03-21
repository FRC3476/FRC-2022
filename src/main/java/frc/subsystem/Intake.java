package frc.subsystem;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.utility.OrangeUtility;
import frc.utility.Timer;
import frc.utility.controllers.LazyCANSparkMax;

import static frc.utility.Pneumatics.getPneumaticsHub;

public final class Intake extends AbstractSubsystem {

    private static Intake instance = new Intake();

    private final Solenoid intakeSol;
    private final LazyCANSparkMax intakeMotor;

    private double allowIntakeRunTime = Double.MAX_VALUE;

    public static Intake getInstance() {
        return instance;
    }

    private Intake() {
        super(Constants.INTAKE_PERIOD, 4);
        intakeSol = getPneumaticsHub().makeSolenoid(Constants.INTAKE_SOLENOID_CHANNEL);
        intakeMotor = new LazyCANSparkMax(Constants.INTAKE_MOTOR_DEVICE_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        intakeMotor.setSmartCurrentLimit(30);
        intakeMotor.setControlFramePeriodMs(25);
    }

    @Override
    public void selfTest() {
        setIntakeSolState(IntakeSolState.OPEN);
        OrangeUtility.sleep(1000);
        setIntakeState(IntakeState.INTAKE);
        OrangeUtility.sleep(3000);
        setIntakeState(IntakeState.OFF);
        setIntakeSolState(IntakeSolState.CLOSE);
    }

    @Override
    public void logData() {
        SmartDashboard.putNumber("Intake Motor Speed: ", intakeMotor.get());
        SmartDashboard.putNumber("Intake Current", intakeMotor.getOutputCurrent());
        SmartDashboard.putBoolean("Intake Solenoid State: ", intakeSol.get());
        logData("Wanted Intake State", wantedIntakeState);
    }


    @Override
    public void close() throws Exception {
        intakeSol.close();
        intakeMotor.close();
        instance = new Intake();
    }

    public IntakeSolState getIntakeSolState() {
        return intakeSol.get() ? IntakeSolState.OPEN : IntakeSolState.CLOSE;
    }

    // Intake States

    public enum IntakeSolState {
        OPEN, CLOSE
    }


    public synchronized void setIntakeSolState(IntakeSolState intakeSolState) {
        SmartDashboard.putString("Intake State", intakeSolState.toString());
        switch (intakeSolState) {
            case OPEN:
                if (Timer.getFPGATimestamp() + Constants.INTAKE_OPEN_TIME < allowIntakeRunTime) {
                    allowIntakeRunTime = Timer.getFPGATimestamp() + Constants.INTAKE_OPEN_TIME;
                }
                intakeSol.set(true);
                break;
            case CLOSE:
                intakeSol.set(false);
                allowIntakeRunTime = Double.MAX_VALUE;
        }
    }

    public enum IntakeState {
        INTAKE, EJECT, OFF
    }

    IntakeState wantedIntakeState = IntakeState.OFF;

    public synchronized void setWantedIntakeState(IntakeState intakeState) {
        wantedIntakeState = intakeState;
    }

    private void setIntakeState(IntakeState intakeState) {
        switch (intakeState) {
            case INTAKE:
                intakeMotor.set(Constants.INTAKE_MOTOR_SPEED);
                break;

            case EJECT:
                intakeMotor.set(-Constants.INTAKE_MOTOR_SPEED);
                break;

            case OFF:
                intakeMotor.set(0);
                break;
        }
    }

    @Override
    public synchronized void update() {
        if (Timer.getFPGATimestamp() > allowIntakeRunTime) {
            setIntakeState(wantedIntakeState);
        } else {
            setIntakeState(IntakeState.OFF);
        }
    }
}