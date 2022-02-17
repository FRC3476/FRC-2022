package frc.utility;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import frc.auton.guiauto.serialization.Autonomous;
import org.jetbrains.annotations.NotNull;

import java.io.File;
import java.io.IOException;

public final class Serializer {
    private static final @NotNull ObjectMapper OBJECT_MAPPER = new ObjectMapper();

    public static String serializeToString(Object obj) throws com.fasterxml.jackson.core.JsonProcessingException {
        OBJECT_MAPPER.configure(SerializationFeature.INDENT_OUTPUT, true);
        return OBJECT_MAPPER.writeValueAsString(obj);
    }

    public static void serializeToFile(Object obj,
                                       File file) throws IOException, com.fasterxml.jackson.core.JsonGenerationException, com.fasterxml.jackson.databind.JsonMappingException {
        OBJECT_MAPPER.configure(SerializationFeature.INDENT_OUTPUT, true);
        OBJECT_MAPPER.writeValue(file, obj);
    }

    public static Autonomous deserializeAutoFromFile(
            File file) throws IOException, com.fasterxml.jackson.core.JsonParseException, com.fasterxml.jackson.databind.JsonMappingException {
        return OBJECT_MAPPER.readValue(file, Autonomous.class);
    }

    public static Object deserialize(String object,
                                     Class<?> serializableObject) throws com.fasterxml.jackson.core.JsonProcessingException, com.fasterxml.jackson.databind.JsonMappingException {
        return OBJECT_MAPPER.readValue(object, serializableObject);
    }

    public static Autonomous deserializeAuto(
            String object) throws ClassNotFoundException, com.fasterxml.jackson.core.JsonProcessingException, com.fasterxml.jackson.databind.JsonMappingException {
        return OBJECT_MAPPER.readValue(object, Autonomous.class);
    }

    public static Object deserializeFromFile(File file,
                                             Class<?> serializableObject) throws IOException, com.fasterxml.jackson.core.JsonParseException, com.fasterxml.jackson.databind.JsonMappingException {
        return OBJECT_MAPPER.readValue(file, serializableObject);
    }
}