package frc.subsystem;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;
import frc.utility.Timer;
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
        Timer.setTime(0);
        intake.setIntakeSolState(Intake.IntakeSolState.CLOSE);
        intake.update();
        Timer.setTime(0.5);
        intake.setWantedIntakeState(Intake.IntakeState.INTAKE);
        intake.update();

        Field intakeMotorField = Intake.class.getDeclaredField("intakeMotor");
        intakeMotorField.setAccessible(true);
        TalonFX intakeMotor = (TalonFX) intakeMotorField.get(intake);

        assertEquals(0, intakeMotor.getMotorOutputPercent(), DELTA);
    }

    public void intakeDoesNotRunReversedWhenClosed() throws Exception {
        Timer.setTime(0);
        intake.setIntakeSolState(Intake.IntakeSolState.CLOSE);
        intake.update();
        Timer.setTime(0.5);
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
        Timer.setTime(1000);
        intake.setIntakeSolState(Intake.IntakeSolState.OPEN);
        intake.update();
        Timer.setTime(1001);
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
        Timer.setTime(0);
        intake.setIntakeSolState(Intake.IntakeSolState.OPEN);
        intake.update();
        Timer.setTime(0.5);
        intake.setWantedIntakeState(Intake.IntakeState.INTAKE);
        intake.update();

        Field intakeMotorField = Intake.class.getDeclaredField("intakeMotorFalcon");
        intakeMotorField.setAccessible(true);
        TalonFX intakeMotor = (TalonFX) intakeMotorField.get(intake);

        assertEquals(Constants.INTAKE_SPEED, intakeMotor.getMotorOutputPercent(), DELTA);
    }
}