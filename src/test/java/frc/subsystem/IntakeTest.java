package frc.subsystem;

import frc.robot.Constants;
import frc.utility.controllers.LazyCANSparkMax;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.lang.reflect.Field;

import static org.junit.jupiter.api.Assertions.assertEquals;

class IntakeTest {
    Intake intake;

    public static final double DELTA = 1.0e-3;

    @BeforeEach
    public void reset() throws Exception {
        Intake.getInstance().close();
        intake = Intake.getInstance();
    }

    @Test
    public void intakeDoesNotRunWhenClosed() throws Exception{
        intake.setIntakeSolState(Intake.IntakeSolState.CLOSE);
        intake.setIntakeState(Intake.IntakeState.INTAKE);

        Field intakeMotorField = Intake.class.getDeclaredField("intakeMotor");
        intakeMotorField.setAccessible(true);
        LazyCANSparkMax intakeMotor = (LazyCANSparkMax) intakeMotorField.get(intake);

        assertEquals(0, intakeMotor.get(), DELTA);
    }

    @Test
    public void intakeDoesNotRunReversedWhenClosed() throws Exception{
        intake.setIntakeSolState(Intake.IntakeSolState.CLOSE);
        intake.setIntakeState(Intake.IntakeState.EJECT);

        Field intakeMotorField = Intake.class.getDeclaredField("intakeMotor");
        intakeMotorField.setAccessible(true);
        LazyCANSparkMax intakeMotor = (LazyCANSparkMax) intakeMotorField.get(intake);

        assertEquals(0, intakeMotor.get(), DELTA);
    }

    @Test
    public void intakeDoesRunsReversedWhenOpen() throws Exception{
        intake.setIntakeSolState(Intake.IntakeSolState.OPEN);
        intake.setIntakeState(Intake.IntakeState.EJECT);

        Field intakeMotorField = Intake.class.getDeclaredField("intakeMotor");
        intakeMotorField.setAccessible(true);
        LazyCANSparkMax intakeMotor = (LazyCANSparkMax) intakeMotorField.get(intake);

        assertEquals(Constants.INTAKE_MOTOR_SPEED, intakeMotor.get(), DELTA);
    }
}