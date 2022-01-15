package frc.auton.guiauto.serialization.command;

import frc.subsystem.Drive;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

class SendableCommandTest {

    @Test
    void testDeserialize() {
        SendableCommand command = new SendableCommand("print",
                new String[]{"5", "Hello World"},
                new String[]{int.class.getName(), String.class.getName()}, false);

        assertNotNull(command.objArgs);
        assertEquals(2, command.objArgs.length);
        assertEquals(5, command.objArgs[0]);
        assertEquals("Hello World", command.objArgs[1]);
    }

    @Test
    void testEnumDeserialize() {
        SendableCommand command = new SendableCommand("sleep",
                new String[]{"DONE", "12.12432", "19"},
                new String[]{Drive.DriveState.class.getName(), Double.class.getName(), short.class.getName()}, false);

        assertNotNull(command.objArgs);
        assertEquals(3, command.objArgs.length);
        assertEquals(Drive.DriveState.DONE, command.objArgs[0]);
        assertEquals(12.12432, command.objArgs[1]);
        assertEquals((short) 19, command.objArgs[2]);
    }

    @Test
    void testReflectionDeserialization() {
        SendableCommand command = new SendableCommand(Drive.class.getName() + ".setDriveState",
                new String[]{"DONE"},
                new String[]{Drive.DriveState.class.getName()}, true);

        assertEquals(1, command.objArgs.length);
        assertEquals(Drive.DriveState.DONE, command.objArgs[0]);
        assertEquals(Drive.getInstance(), command.instance);
    }

    @Test
    void noArgsMethodDeserialization() {
        SendableCommand command = new SendableCommand("frc.subsystem.Drive.configBrake",
                new String[]{},
                new String[]{}, true);

        assertEquals(0, command.objArgs.length);
        assertEquals(Drive.getInstance(), command.instance);
    }
}