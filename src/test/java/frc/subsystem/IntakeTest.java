package frc.subsystem;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.util.WPIUtilJNI;
import frc.robot.Constants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;

import java.lang.reflect.Field;

import static org.junit.jupiter.api.Assertions.assertEquals;

class IntakeTest {
    Intake intake;

    public static final double DELTA = 1.0e-3;

    @BeforeEach
    public void reset() throws Exception {
        intake = Intake.getInstance();
    }

    public void intakeDoesNotRunWhenClosed() throws Exception {
        WPIUtilJNI.enableMockTime();
        WPIUtilJNI.setMockTime(0);
        intake.setIntakeSolState(Intake.IntakeSolState.CLOSE);
        intake.update();
        WPIUtilJNI.setMockTime((long) (0.5d * 1.0e+9));
        intake.setWantedIntakeState(Intake.IntakeState.INTAKE);
        intake.update();

        Field intakeMotorField = Intake.class.getDeclaredField("intakeMotor");
        intakeMotorField.setAccessible(true);
        TalonFX intakeMotor = (TalonFX) intakeMotorField.get(intake);

        assertEquals(0, intakeMotor.getMotorOutputPercent(), DELTA);
    }

    public void intakeDoesNotRunReversedWhenClosed() throws Exception {
        WPIUtilJNI.enableMockTime();
        WPIUtilJNI.setMockTime(0);
        intake.setIntakeSolState(Intake.IntakeSolState.CLOSE);
        intake.update();
        WPIUtilJNI.setMockTime((long) (0.5d * 1.0e+9));
        intake.setWantedIntakeState(Intake.IntakeState.EJECT);
        intake.update();
        Field intakeMotorField = Intake.class.getDeclaredField("intakeMotorFalcon");
        intakeMotorField.setAccessible(true);
        TalonFX intakeMotor = (TalonFX) intakeMotorField.get(intake);

        assertEquals(0, intakeMotor.getMotorOutputPercent(), DELTA);
    }

    @Disabled
    public void intakeDoesRunReversedWhenOpen() throws Exception {
        reset();
        WPIUtilJNI.enableMockTime();
        WPIUtilJNI.setMockTime((long) (1000.0d * 1.0e+9));
        intake.setIntakeSolState(Intake.IntakeSolState.OPEN);
        intake.update();
        WPIUtilJNI.setMockTime((long) (1001.0d * 1.0e+9));
        intake.setWantedIntakeState(Intake.IntakeState.EJECT);
        intake.update();
        intake.update();

        Field intakeMotorField = Intake.class.getDeclaredField("intakeMotorFalcon");
        intakeMotorField.setAccessible(true);
        TalonFX intakeMotor = (TalonFX) intakeMotorField.get(intake);

        assertEquals(-Constants.INTAKE_SPEED, intakeMotor.getMotorOutputPercent(), DELTA);
    }

    @Disabled
    public void intakeDoesRunWhenOpen() throws Exception {
        WPIUtilJNI.enableMockTime();
        WPIUtilJNI.setMockTime(0);
        intake.setIntakeSolState(Intake.IntakeSolState.OPEN);
        intake.update();
        WPIUtilJNI.setMockTime((long) (0.5d * 1.0e+9));
        intake.setWantedIntakeState(Intake.IntakeState.INTAKE);
        intake.update();

        Field intakeMotorField = Intake.class.getDeclaredField("intakeMotorFalcon");
        intakeMotorField.setAccessible(true);
        TalonFX intakeMotor = (TalonFX) intakeMotorField.get(intake);

        assertEquals(Constants.INTAKE_SPEED, intakeMotor.getMotorOutputPercent(), DELTA);
    }
}