package frc.auton.guiauto.serialization;

import edu.wpi.first.wpilibj.DriverStation;
import frc.auton.TemplateAuto;
import frc.utility.OrangeUtility;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.function.Function;

public class Parser {
    private static final ArrayList<Function<String, Object>> TRY_PARSE_LIST;

    static {
        TRY_PARSE_LIST = new ArrayList<>();
        TRY_PARSE_LIST.add(Integer::parseInt);
        TRY_PARSE_LIST.add(Double::parseDouble);
        TRY_PARSE_LIST.add(s -> {
            if (s.equalsIgnoreCase("true")) {
                return true;
            } else if (s.equalsIgnoreCase("false")) return false;
            throw new IllegalArgumentException();
        });
        TRY_PARSE_LIST.add(s -> s);
    }

    public static boolean execute(String string, TemplateAuto context) {
        String[] commands = string.split("\n"); //Start by splitting everything by lines (commands)
        for (String command : commands) {
            if (command.startsWith("#")) continue; //Ignore comments
            String[] parts = command.split(" "); //Split by spaces
            if (parts.length == 0) continue; //Ignore empty lines
            String method = parts[0]; //Get the method name
            String[] args = new String[parts.length - 1]; //Initialize the argument array
            System.arraycopy(parts, 1, args, 0, parts.length - 1);

            try {
                switch (method) {
                    case "print":
                        StringBuilder aggregate = new StringBuilder();
                        for (String arg : args) {
                            aggregate.append(arg).append(" ");
                        }
                        System.out.println(aggregate);
                        break;
                    case "sleep":
                        OrangeUtility.sleep(Long.parseLong(args[0]));
                        break;
                    default:
                        //Try using reflection
                        try {
                            String[] splitMethod = method.split("\\.");
                            Class<?> cls = Class.forName("java.frc.subsystem." + splitMethod[0]);

                            Object[] objArgs = new Object[args.length];
                            for (int i = 0; i < args.length; i++) {
                                for (Function<String, Object> parser : TRY_PARSE_LIST) {
                                    try {
                                        objArgs[i] = parser.apply(args[i]);
                                        break;
                                    } catch (IllegalArgumentException e) {
                                        continue;
                                    }
                                }
                            }

                            Class<?>[] argClasses = new Class[objArgs.length];
                            for (int i = 0; i < objArgs.length; i++) {
                                argClasses[i] = getPrimitiveClass(objArgs[i].getClass());
                            }

                            Object instance = cls.getMethod("getInstance").invoke(null);
                            Method javaMethod = cls.getMethod(splitMethod[1], argClasses);

                            javaMethod.invoke(instance, objArgs);
                            break;
                        } catch (Exception e) {
                            DriverStation.reportError("Issue while trying to invoke method using reflection", e.getStackTrace());
                            return false;
                        }
                }
            } catch (Exception e) {
                e.printStackTrace();
                return false;
            }
        }
        return true;
    }

    private static Class<?> getPrimitiveClass(Class<?> cls) {
        if (cls.equals(Integer.class)) return int.class;
        if (cls.equals(Double.class)) return double.class;
        if (cls.equals(Boolean.class)) return boolean.class;
        return cls;
    }
}