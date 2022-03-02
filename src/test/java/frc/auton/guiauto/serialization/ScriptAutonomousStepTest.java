package frc.auton.guiauto.serialization;

import frc.auton.TemplateAuto;
import frc.auton.guiauto.serialization.command.SendableCommand;
import frc.auton.guiauto.serialization.command.SendableScript;
import frc.subsystem.Drive;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

class ScriptAutonomousStepTest {

    @BeforeAll
    static void setUp() throws Exception {
        Drive.getInstance().kill();

    }

    @BeforeEach
    void setUpBeforeEach() {
        Drive.getInstance().setDriveState(Drive.DriveState.DONE);
    }

    @Test
    void execute() {
    }

    @Test
    void testTimeDelayedScriptExecution() throws Exception {
        List<SendableCommand> commandList = new ArrayList<>();
        commandList.add(new SendableCommand(Drive.class.getName() + ".setDriveState", new String[]{"TELEOP"},
                new String[]{Drive.DriveState.class.getName()}, true));

        SendableScript sendableScript = new SendableScript(SendableScript.DelayType.TIME, 10, commandList);

        ScriptAutonomousStep scriptAutonomousStep = new ScriptAutonomousStep(sendableScript);

        List<SendableScript> timeDelayedCommandList = new ArrayList<>();
        List<SendableScript> percentDelayedCommandList = new ArrayList<>();

        scriptAutonomousStep.execute(new TemplateAuto() {
            @Override
            public void run() {

            }
        }, timeDelayedCommandList, percentDelayedCommandList);


        Field driveState = Drive.class.getDeclaredField("driveState"); //Use reflection to access private field
        driveState.setAccessible(true);
        assertEquals(Drive.DriveState.DONE, driveState.get(Drive.getInstance()));

        assertEquals(1, timeDelayedCommandList.size());
        assertEquals(0, percentDelayedCommandList.size());
        assertSame(sendableScript, timeDelayedCommandList.get(0));
    }

    @Test
    void testPercentDelayedScriptExecution() throws Exception {
        List<SendableCommand> commandList = new ArrayList<>();
        commandList.add(new SendableCommand(Drive.class.getName() + ".setDriveState", new String[]{"TELEOP"},
                new String[]{Drive.DriveState.class.getName()}, true));

        SendableScript sendableScript = new SendableScript(SendableScript.DelayType.PERCENT, 10, commandList);

        ScriptAutonomousStep scriptAutonomousStep = new ScriptAutonomousStep(sendableScript);

        List<SendableScript> timeDelayedCommandList = new ArrayList<>();
        List<SendableScript> percentDelayedCommandList = new ArrayList<>();

        scriptAutonomousStep.execute(new TemplateAuto() {
            @Override
            public void run() {

            }
        }, timeDelayedCommandList, percentDelayedCommandList);

        Field driveState = Drive.class.getDeclaredField("driveState"); //Use reflection to access private field
        driveState.setAccessible(true);
        assertEquals(Drive.DriveState.DONE, driveState.get(Drive.getInstance()));

        assertEquals(0, timeDelayedCommandList.size());
        assertEquals(1, percentDelayedCommandList.size());
        assertSame(sendableScript, percentDelayedCommandList.get(0));
    }

    @Test
    void testNoDelayScriptExecution() throws Exception {
        List<SendableCommand> commandList = new ArrayList<>();
        commandList.add(new SendableCommand(Drive.class.getName() + ".setDriveState", new String[]{"TELEOP"},
                new String[]{Drive.DriveState.class.getName()}, true));

        SendableScript sendableScript = new SendableScript(SendableScript.DelayType.NONE, 10, commandList);

        ScriptAutonomousStep scriptAutonomousStep = new ScriptAutonomousStep(sendableScript);

        List<SendableScript> timeDelayedCommandList = new ArrayList<>();
        List<SendableScript> percentDelayedCommandList = new ArrayList<>();

        scriptAutonomousStep.execute(new TemplateAuto() {
            @Override
            public void run() {

            }
        }, timeDelayedCommandList, percentDelayedCommandList);

        Field driveState = Drive.class.getDeclaredField("driveState"); //Use reflection to access private field
        driveState.setAccessible(true);
        assertEquals(Drive.DriveState.TELEOP, driveState.get(Drive.getInstance()));

        assertEquals(0, timeDelayedCommandList.size());
        assertEquals(0, percentDelayedCommandList.size());
    }
}