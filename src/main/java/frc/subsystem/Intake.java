package frc.subsystem;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.utility.OrangeUtility;
import frc.utility.Timer;
import frc.utility.controllers.LazyCANSparkMax;

public class Intake extends AbstractSubsystem {

    private static Intake instance = new Intake();

    private final Solenoid intakeSol;
    private LazyCANSparkMax intakeMotor;

    private double allowIntakeRunTime = 0;

    public static Intake getInstance() {
        return instance;
    }

    private Intake() {
        super(100);
        intakeSol = new Solenoid(PneumaticsModuleType.REVPH, Constants.SOLENOID_CHANNEL);
        intakeMotor = new LazyCANSparkMax(Constants.INTAKE_MOTOR_DEVICE_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
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
        SmartDashboard.putBoolean("Intake Solenoid State: ", intakeSol.get());
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

    enum IntakeSolState {
        OPEN, CLOSE
    }

    public void setIntakeSolState(IntakeSolState intakeSolState) {
        switch (intakeSolState) {
            case OPEN:
                if (getIntakeSolState() == IntakeSolState.CLOSE) {
                    allowIntakeRunTime = Timer.getFPGATimestamp() + Constants.INTAKE_OPEN_TIME;
                }
                intakeSol.set(true);
                break;
            case CLOSE:
                intakeSol.set(false);
        }
    }

    enum IntakeState {
        INTAKE, EJECT, OFF
    }

    IntakeState wantedIntakeState = IntakeState.OFF;

    public void setWantedIntakeState(IntakeState intakeState) {
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
    public void update() {
        if (Timer.getFPGATimestamp() > allowIntakeRunTime && getIntakeSolState() == IntakeSolState.OPEN) {
            setIntakeState(wantedIntakeState);
        } else {
            setIntakeState(IntakeState.OFF);
        }
    }
}