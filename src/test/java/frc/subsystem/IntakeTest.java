package frc.subsystem;

import frc.robot.Constants;
import frc.utility.Timer;
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

    public void intakeDoesNotRunWhenClosed() throws Exception {
        Timer.setTime(0);
        intake.setIntakeSolState(Intake.IntakeSolState.CLOSE);
        intake.update();
        Timer.setTime(0.5);
        intake.setWantedIntakeState(Intake.IntakeState.INTAKE);
        intake.update();

        Field intakeMotorField = Intake.class.getDeclaredField("intakeMotor");
        intakeMotorField.setAccessible(true);
        LazyCANSparkMax intakeMotor = (LazyCANSparkMax) intakeMotorField.get(intake);

        assertEquals(0, intakeMotor.get(), DELTA);
    }

    public void intakeDoesNotRunReversedWhenClosed() throws Exception {
        Timer.setTime(0);
        intake.setIntakeSolState(Intake.IntakeSolState.CLOSE);
        intake.update();
        Timer.setTime(0.5);
        intake.setWantedIntakeState(Intake.IntakeState.EJECT);
        intake.update();
        Field intakeMotorField = Intake.class.getDeclaredField("intakeMotor");
        intakeMotorField.setAccessible(true);
        LazyCANSparkMax intakeMotor = (LazyCANSparkMax) intakeMotorField.get(intake);

        assertEquals(0, intakeMotor.get(), DELTA);
    }

    @Test
    public void intakeDoesRunReversedWhenOpen() throws Exception {
        reset();
        Timer.setTime(1000);
        intake.setIntakeSolState(Intake.IntakeSolState.OPEN);
        intake.update();
        Timer.setTime(1001);
        intake.setWantedIntakeState(Intake.IntakeState.EJECT);
        intake.update();
        intake.update();

        Field intakeMotorField = Intake.class.getDeclaredField("intakeMotor");
        intakeMotorField.setAccessible(true);
        LazyCANSparkMax intakeMotor = (LazyCANSparkMax) intakeMotorField.get(intake);

        assertEquals(-Constants.INTAKE_MOTOR_SPEED, intakeMotor.get(), DELTA);
    }

    @Test
    public void intakeDoesRunWhenOpen() throws Exception {
        Timer.setTime(0);
        intake.setIntakeSolState(Intake.IntakeSolState.OPEN);
        intake.update();
        Timer.setTime(0.5);
        intake.setWantedIntakeState(Intake.IntakeState.INTAKE);
        intake.update();

        Field intakeMotorField = Intake.class.getDeclaredField("intakeMotor");
        intakeMotorField.setAccessible(true);
        LazyCANSparkMax intakeMotor = (LazyCANSparkMax) intakeMotorField.get(intake);

        assertEquals(Constants.INTAKE_MOTOR_SPEED, intakeMotor.get(), DELTA);
    }
}