package frc.subsystem;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.subsystem.Hopper.OuttakeState;
import frc.utility.OrangeUtility;
import frc.utility.Timer;

import static frc.utility.Pneumatics.getPneumaticsHub;

public final class Intake extends AbstractSubsystem {

    private static Intake instance = new Intake();

    private final Solenoid intakeSol;
    private final TalonFX intakeMotor;

    private double allowIntakeRunTime = Double.MAX_VALUE;

    public static Intake getInstance() {
        return instance;
    }

    private Intake() {
        super(Constants.INTAKE_PERIOD, 4);
        intakeSol = getPneumaticsHub().makeSolenoid(Constants.INTAKE_SOLENOID_CHANNEL);
        intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_DEVICE_ID);

        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 97); // Default is 10ms
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 101); // Default is 10ms
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 103); // Default is 50ms
        intakeMotor.setControlFramePeriod(ControlFrame.Control_3_General, 23);
        intakeMotor.setControlFramePeriod(ControlFrame.Control_4_Advanced, 29);
        intakeMotor.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 547);
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
        SmartDashboard.putNumber("Intake Motor Speed: ", intakeMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Intake Current", intakeMotor.getStatorCurrent());
        SmartDashboard.putBoolean("Intake Solenoid State: ", intakeSol.get());
        logData("Wanted Intake State", wantedIntakeState);
    }


    @Override
    public void close() throws Exception {
        intakeSol.close();
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

    private IntakeState wantedIntakeState = IntakeState.OFF;

    public synchronized void setWantedIntakeState(IntakeState intakeState) {
        wantedIntakeState = intakeState;
    }

    private void setIntakeState(IntakeState intakeState) {
        switch (intakeState) {
            case INTAKE:
                if (Hopper.getInstance().getOuttakeState() == OuttakeState.AUTO_EJECT) {
                    intakeMotor.set(ControlMode.PercentOutput, Constants.INTAKE_EJECTION_SPEED);
                } else {
                    intakeMotor.set(ControlMode.PercentOutput, Constants.INTAKE_SPEED);
                }
                break;

            case EJECT:
                intakeMotor.set(ControlMode.PercentOutput, -Constants.INTAKE_SPEED);
                break;

            case OFF:
                intakeMotor.set(ControlMode.PercentOutput, 0);
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