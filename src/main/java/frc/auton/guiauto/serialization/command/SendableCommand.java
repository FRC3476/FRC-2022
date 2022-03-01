package frc.auton.guiauto.serialization.command;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.wpilibj.DriverStation;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

@JsonIgnoreProperties(ignoreUnknown = true)
public class SendableCommand {

    @JsonProperty("methodName")
    public final @NotNull String methodName;

    @JsonProperty("args") public final String @NotNull [] args;

    @JsonProperty("argTypes") public final String[] argTypes;

    @JsonProperty("reflection") public final boolean reflection;

    private static final @NotNull Map<String, Function<String, Object>> INFERABLE_TYPES_PARSER;

    static {
        INFERABLE_TYPES_PARSER = new HashMap<>();
        INFERABLE_TYPES_PARSER.put(int.class.getName(), Integer::parseInt);
        INFERABLE_TYPES_PARSER.put(double.class.getName(), Double::parseDouble);
        INFERABLE_TYPES_PARSER.put(float.class.getName(), Float::parseFloat);
        INFERABLE_TYPES_PARSER.put(long.class.getName(), Long::parseLong);
        INFERABLE_TYPES_PARSER.put(short.class.getName(), Short::parseShort);
        INFERABLE_TYPES_PARSER.put(byte.class.getName(), Byte::parseByte);
        INFERABLE_TYPES_PARSER.put(char.class.getName(), s -> s.charAt(0));
        INFERABLE_TYPES_PARSER.put(boolean.class.getName(), Boolean::parseBoolean);
        INFERABLE_TYPES_PARSER.put(String.class.getName(), s -> s);
        INFERABLE_TYPES_PARSER.put(Integer.class.getName(), Integer::valueOf);
        INFERABLE_TYPES_PARSER.put(Double.class.getName(), Double::valueOf);
        INFERABLE_TYPES_PARSER.put(Float.class.getName(), Float::valueOf);
        INFERABLE_TYPES_PARSER.put(Long.class.getName(), Long::valueOf);
        INFERABLE_TYPES_PARSER.put(Short.class.getName(), Short::valueOf);
        INFERABLE_TYPES_PARSER.put(Byte.class.getName(), Byte::valueOf);
        INFERABLE_TYPES_PARSER.put(Character.class.getName(), s -> Character.valueOf(s.charAt(0)));
        INFERABLE_TYPES_PARSER.put(Boolean.class.getName(), Boolean::valueOf);
    }

    @JsonCreator
    public SendableCommand(@JsonProperty("methodName") @NotNull String methodName,
                           @JsonProperty("args") String @NotNull [] args,
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
                if (INFERABLE_TYPES_PARSER.containsKey(argTypes[i])) {
                    objArgs[i] = INFERABLE_TYPES_PARSER.get(argTypes[i]).apply(args[i]);
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

            String[] classNameArray = new String[splitMethod.length - 1];
            System.arraycopy(splitMethod, 0, classNameArray, 0, classNameArray.length);
            String className = String.join(".", classNameArray);

            try {
                Class<?> cls = Class.forName(className);
                Class<?>[] typeArray = Arrays.stream(objArgs).sequential().map(Object::getClass).toArray(Class[]::new);
                if (typeArray.length == 0) {
                    methodToCall = cls.getDeclaredMethod(splitMethod[splitMethod.length - 1]);
                } else {
                    methodToCall = cls.getDeclaredMethod(splitMethod[splitMethod.length - 1],
                            Arrays.stream(objArgs).sequential().map((o) -> getPrimitiveClass(o.getClass()))
                                    .toArray(Class<?>[]::new));
                }
                methodToCall.setAccessible(true);
                if (!Modifier.isStatic(methodToCall.getModifiers())) {
                    Method getInstance = cls.getDeclaredMethod("getInstance");
                    getInstance.setAccessible(true);
                    instance = getInstance.invoke(null);
                }
            } catch (ClassNotFoundException e) {
                DriverStation.reportError("Class not found: " + className + ". " + e.getMessage(), e.getStackTrace());
            } catch (NoSuchMethodException e) {
                DriverStation.reportError(
                        "Could not find method : " + splitMethod[splitMethod.length - 1] + " in class " + className + ". " + e.getMessage(),
                        e.getStackTrace());
            } catch (InvocationTargetException | IllegalAccessException e) {
                DriverStation.reportError("Could not get singleton reference in class " + className + " for method: " +
                        splitMethod[splitMethod.length - 1] + ". " + e.getMessage(), e.getStackTrace());
            }
        }

        this.methodToCall = methodToCall;
        this.instance = instance;
    }

    @JsonIgnoreProperties
    final @Nullable Object instance;

    @JsonIgnoreProperties
    final @Nullable Method methodToCall;

    @JsonIgnoreProperties final Object @NotNull [] objArgs;

    private static Class<?> getPrimitiveClass(Class<?> clazz) {
        if (clazz.equals(Integer.class)) {
            return double.class;
        } else if (clazz.equals(Double.class)) {
            return double.class;
        } else if (clazz.equals(Boolean.class)) {
            return boolean.class;
        } else if (clazz.equals(char.class)) {
            return char.class;
        } else if (clazz.equals(Byte.class)) {
            return byte.class;
        } else if (clazz.equals(Short.class)) {
            return short.class;
        } else if (clazz.equals(Long.class)) {
            return long.class;
        } else if (clazz.equals(Float.class)) {
            return float.class;
        } else if (clazz.equals(String.class)) {
            return String.class;
        } else {
            return clazz;
        }
    }


    /**
     * @throws InterruptedException            If the command fails do to an interrupt
     * @throws CommandExecutionFailedException If the command fails to execute for any other reason
     */
    public void execute() throws InterruptedException, CommandExecutionFailedException {
        if (methodToCall == null && reflection) {
            DriverStation.reportError("Method to call is null", Thread.currentThread().getStackTrace());
            throw new CommandExecutionFailedException("Method to call is null");
        }
        try {
            if (reflection) {
                methodToCall.invoke(instance, objArgs);
            } else {
                switch (methodName) {
                    case "print":
                        System.out.println(objArgs[0]);
                        break;
                    case "sleep":
                        Thread.sleep((long) objArgs[0]);
                        break;
                }
            }
        } catch (InterruptedException e) {
            throw new InterruptedException("Interrupted while executing a script");
        } catch (Exception e) {
            DriverStation.reportError("Could not invoke method " + methodName + " due to: " + e.getMessage(),
                    e.getStackTrace());
            throw new CommandExecutionFailedException("Could not invoke method " + methodName + " due to: " + e.getMessage(), e);
        }
    }
}
