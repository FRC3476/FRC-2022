package frc.subsystem;

import com.fasterxml.jackson.core.JsonProcessingException;
import frc.utility.Serializer;
import org.junit.jupiter.api.Test;

import java.lang.reflect.Field;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;

class DashboardHandlerTest {

    @Test
    public void test() throws NoSuchFieldException, IllegalAccessException, JsonProcessingException {
        Field logDataMap = DashboardHandler.class.getDeclaredField("LOG_DATA_MAP");
        logDataMap.setAccessible(true);
        Map<String, String> hashMap = (Map<String, String>) logDataMap.get(DashboardHandler.getInstance());
        DashboardHandler.getInstance().log("This is a key", "This is a value");
        DashboardHandler.getInstance().log("This is a key2", "This is a value2");
        DashboardHandler.getInstance().log("Subsystem 1", "value1");
        DashboardHandler.getInstance().log("Subsystem 2", "value2");
        DashboardHandler.getInstance().log("Subsystem 3", "value3");
        DashboardHandler.getInstance().log("Subsystem 4", "value4");
        DashboardHandler.getInstance().log("Subsystem 5", "value5");
        DashboardHandler.getInstance().log("Subsystem 6", "value6");
        DashboardHandler.getInstance().log("Subsystem 7", "value7");
        DashboardHandler.getInstance().log("Subsystem 8", "value8");
        DashboardHandler.getInstance().log("Subsystem 9", "value9");
        DashboardHandler.getInstance().log("Subsystem 10.subtable", "value10");
        assertEquals("String", Serializer.serializeToString(hashMap));
    }
}