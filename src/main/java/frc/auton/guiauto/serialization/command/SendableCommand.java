package frc.auton.guiauto.serialization.command;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.wpilibj.DriverStation;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

@JsonIgnoreProperties(ignoreUnknown = true)
public class SendableCommand {

    @JsonProperty("methodName") public final String methodName;

    @JsonProperty("args") public final String[] args;

    @JsonProperty("argTypes") public final String[] argTypes;

    @JsonProperty("reflection") public final boolean reflection;

    public static Map<String, Function<String, Object>> inferableTypesParser;

    static {
        inferableTypesParser = new HashMap<>();
        inferableTypesParser.put(int.class.getName(), Integer::parseInt);
        inferableTypesParser.put(double.class.getName(), Double::parseDouble);
        inferableTypesParser.put(float.class.getName(), Float::parseFloat);
        inferableTypesParser.put(long.class.getName(), Long::parseLong);
        inferableTypesParser.put(short.class.getName(), Short::parseShort);
        inferableTypesParser.put(byte.class.getName(), Byte::parseByte);
        inferableTypesParser.put(char.class.getName(), s -> s.charAt(0));
        inferableTypesParser.put(boolean.class.getName(), Boolean::parseBoolean);
        inferableTypesParser.put(String.class.getName(), s -> s);
        inferableTypesParser.put(Integer.class.getName(), Integer::valueOf);
        inferableTypesParser.put(Double.class.getName(), Double::valueOf);
        inferableTypesParser.put(Float.class.getName(), Float::valueOf);
        inferableTypesParser.put(Long.class.getName(), Long::valueOf);
        inferableTypesParser.put(Short.class.getName(), Short::valueOf);
        inferableTypesParser.put(Byte.class.getName(), Byte::valueOf);
        inferableTypesParser.put(Character.class.getName(), s -> Character.valueOf(s.charAt(0)));
        inferableTypesParser.put(Boolean.class.getName(), Boolean::valueOf);
    }


    @JsonCreator
    public SendableCommand(@JsonProperty("methodName") String methodName,
                           @JsonProperty("args") String[] args,
                           @JsonProperty("argTypes") String[] argTypes,
                           @JsonProperty("reflection") boolean reflection) {
        Method methodToCall = null;
        Object instance = null;

        this.methodName = methodName;
        this.args = args;
        this.argTypes = argTypes;
        this.reflection = reflection;

        objArgs = new Object[args.length];

        for (int i = 0; i < args.length; i++) {
            try {
                if (inferableTypesParser.containsKey(argTypes[i])) {
                    objArgs[i] = inferableTypesParser.get(argTypes[i]).apply(args[i]);
                } else {
                    objArgs[i] = Enum.valueOf(Class.forName(argTypes[i]).asSubclass(Enum.class), args[i]);
                }

            } catch (ClassNotFoundException e) {
                DriverStation.reportError("Class not found: " + argTypes[i], e.getStackTrace());
            } catch (ClassCastException e) {
                DriverStation.reportError("Could not cast " + argTypes[i] + " to an enum", e.getStackTrace());
            } finally {
                if (objArgs[i] == null) { // We failed to parse the argument
                    objArgs[i] = null;
                }
            }
        }

        if (reflection) {
            String[] splitMethod = methodName.split("\\.");
            StringBuilder className = new StringBuilder();
            for (int i = 0; i < splitMethod.length - 1; i++) {
                className.append(splitMethod[i]);
            }

            try {
                Class<?> cls = Class.forName(className.toString());
                methodToCall = cls.getMethod(splitMethod[splitMethod.length - 1],
                        Arrays.stream(objArgs).sequential().map(Object::getClass).toArray(Class[]::new));
                if (!Modifier.isStatic(methodToCall.getModifiers())) {
                    instance = cls.getMethod("getInstance").invoke(null);
                }
            } catch (ClassNotFoundException e) {
                DriverStation.reportError("Class not found: " + className, e.getStackTrace());
            } catch (NoSuchMethodException e) {
                DriverStation.reportError("Could not find method : " + splitMethod[splitMethod.length - 1] + " in class " + className, e.getStackTrace());
            } catch (InvocationTargetException | IllegalAccessException e) {
                DriverStation.reportError("Could not get singleton reference in class " + className + " for method: " +
                        splitMethod[splitMethod.length - 1], e.getStackTrace());
            }
        }

        this.methodToCall = methodToCall;
        this.instance = instance;
    }

    private final Object instance;
    private final Method methodToCall;
    private final Object[] objArgs;


    /**
     * @return false if the command fails to execute
     */
    public boolean execute() {
        if (reflection) {
            try {
                methodToCall.invoke(instance, objArgs);
            } catch (IllegalAccessException e) {
                DriverStation.reportError("Could not access method " + methodName, e.getStackTrace());
                return false;
            } catch (InvocationTargetException e) {
                DriverStation.reportError("Method: " + methodName + " threw an exception while being invoked", e.getStackTrace());
                return false;
            }
        } else {
            try {
                switch (methodName) {
                    case "print":
                        System.out.println(objArgs[0]);
                    case "sleep":
                        Thread.sleep((long) objArgs[0]);
                }
            } catch (InterruptedException e) {
                DriverStation.reportError("Thread interrupted while sleeping", e.getStackTrace());
                return false;
            }
        }
        return true;
    }
}
