package frc.auton.guiauto.serialization.command;

import frc.subsystem.Drive;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

class SendableScriptTest {

    @BeforeAll
    static void setUp() throws Exception {
        Drive.getInstance().kill();

    }

    @BeforeEach
    void setUpBeforeEach() {
        Drive.getInstance().setDriveState(Drive.DriveState.DONE);
    }

    @Test
    void testDeserialization() {
        List<SendableCommand> commandList = new ArrayList<>();
        commandList.add(new SendableCommand("sleep", new String[]{"DONE", "12.12432", "19"},
                new String[]{Drive.DriveState.class.getName(), Double.class.getName(), short.class.getName()}, false));
        commandList.add(new SendableCommand("print", new String[]{"5", "Hello World"},
                new String[]{int.class.getName(), String.class.getName()}, false));

        SendableScript sendableScript = new SendableScript(SendableScript.DelayType.NONE, 0, commandList);

        assertEquals(SendableScript.DelayType.NONE, sendableScript.getDelayType());
        assertEquals(0, sendableScript.getDelay());
        assertEquals(2, sendableScript.getCommands().size());
        assertEquals(commandList, sendableScript.getCommands());
    }

    @Disabled
    void testScriptExecution() throws NoSuchFieldException, IllegalAccessException {
//        List<SendableCommand> commandList = new ArrayList<>();
//        commandList.add(new SendableCommand(Drive.class.getName() + ".setDriveState", new String[]{"TELEOP"},
//                new String[]{Drive.DriveState.class.getName()}, true));
//
//        SendableScript sendableScript = new SendableScript(SendableScript.DelayType.NONE, 0, commandList);
//
//        assertTrue(sendableScript.execute());
//
//        Field driveState = Drive.class.getDeclaredField("driveState"); //Use reflection to access private field
//        driveState.setAccessible(true);
//        assertEquals(Drive.DriveState.TELEOP, driveState.get(Drive.getInstance()));
    }
}