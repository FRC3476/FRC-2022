package frc.subsystem;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.subsystem.Hopper.OuttakeState;
import frc.utility.OrangeUtility;
import frc.utility.Timer;
import frc.utility.controllers.LazyCANSparkMax;

import static frc.utility.Pneumatics.getPneumaticsHub;

public final class Intake extends AbstractSubsystem {

    private static Intake instance = new Intake();

    private final Solenoid intakeSol;
    private LazyCANSparkMax intakeMotorSpark;
    private TalonFX intakeMotorFalcon;

    private double allowIntakeRunTime = Double.MAX_VALUE;

    public static Intake getInstance() {
        return instance;
    }

    private Intake() {
        super(Constants.INTAKE_PERIOD, 4);
        intakeSol = getPneumaticsHub().makeSolenoid(Constants.INTAKE_SOLENOID_CHANNEL);
        if (Constants.IS_PRACTICE) {
            intakeMotorSpark = new LazyCANSparkMax(Constants.INTAKE_MOTOR_DEVICE_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
            intakeMotorSpark.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
            intakeMotorSpark.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
            intakeMotorSpark.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
            intakeMotorSpark.setSmartCurrentLimit(30);
            intakeMotorSpark.setControlFramePeriodMs(25);
        } else {
            intakeMotorFalcon = new TalonFX(Constants.INTAKE_MOTOR_DEVICE_ID);

            intakeMotorFalcon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 97); // Default is 10ms
            intakeMotorFalcon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 101); // Default is 10ms
            intakeMotorFalcon.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 103); // Default is 50ms
            intakeMotorFalcon.setControlFramePeriod(ControlFrame.Control_3_General, 23);
            intakeMotorFalcon.setControlFramePeriod(ControlFrame.Control_4_Advanced, 29);
            intakeMotorFalcon.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 547);
        }
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
        if (Constants.IS_PRACTICE) {
            SmartDashboard.putNumber("Intake Current", intakeMotorSpark.getOutputCurrent());
            SmartDashboard.putNumber("Intake Motor Speed: ", intakeMotorSpark.get());
        } else {
            SmartDashboard.putNumber("Intake Current", intakeMotorFalcon.getStatorCurrent());
            SmartDashboard.putNumber("Intake Motor Speed: ", intakeMotorFalcon.getMotorOutputPercent());
        }
        SmartDashboard.putBoolean("Intake Solenoid State: ", intakeSol.get());
        logData("Wanted Intake State", wantedIntakeState);
    }


    @Override
    public void close() throws Exception {
        intakeSol.close();
        intakeMotorSpark.close();
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

    private void setIntakeMotor(double speed) {
        if (Constants.IS_PRACTICE) {
            intakeMotorSpark.set(speed);
        } else {
            intakeMotorFalcon.set(ControlMode.PercentOutput, speed);
        }
    }

    private void setIntakeState(IntakeState intakeState) {
        switch (intakeState) {
            case INTAKE:
                if (Hopper.getInstance().getOuttakeState() == OuttakeState.AUTO_EJECT) {
                    setIntakeMotor(Constants.INTAKE_EJECTION_SPEED);
                } else {
                    setIntakeMotor(Constants.INTAKE_SPEED);
                }
                break;

            case EJECT:
                setIntakeMotor(-Constants.INTAKE_SPEED);
                break;

            case OFF:
                setIntakeMotor(0);
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