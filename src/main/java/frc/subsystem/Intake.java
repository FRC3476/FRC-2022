package frc.subsystem;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.utility.controllers.LazyCANSparkMax;

public class Intake extends AbstractSubsystem {

    private static Intake instance = new Intake();

    private final Solenoid intakeSol;
    private LazyCANSparkMax intakeMotor;

    public static Intake getInstance() {
        return instance;
    }

    private Intake() {
        super(-1);
        intakeSol =  new Solenoid(PneumaticsModuleType.CTREPCM, Constants.SOLENOID_CHANNEL);
        intakeMotor= new LazyCANSparkMax(Constants.INTAKE_MOTOR_DEVICE_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    }

    @Override
    public void selfTest() {

    }

    @Override
    public void logData() {

    }

    @Override
    public void close() throws Exception {

    }


    // Intake States

    enum IntakeSolState {
        OPEN, CLOSE
    }

    public void setIntakeSolState(IntakeSolState intakeSolState) {
        switch (intakeSolState) {
            case OPEN:
                intakeSol.set(true);
                break;
            case CLOSE:
                intakeSol.set(false);
        }
    }

    enum IntakeState {
        INTAKE, EJECT, OFF
    }
    public void setIntakeState(IntakeState intakeState) {
        if (intakeSol.get()) {
            switch(intakeState) {
                case INTAKE:
                    setIntakeSolState(IntakeSolState.OPEN);
                    intakeMotor.set(0.6);
                    break;

                case EJECT:
                    setIntakeSolState(IntakeSolState.OPEN);
                    intakeMotor.set(-0.6);
                    break;

                case OFF:
                    setIntakeSolState(IntakeSolState.CLOSE);
                    intakeMotor.set(0);
                    break;
            }
        } else {
            intakeMotor.set(0);
        }

    }
}