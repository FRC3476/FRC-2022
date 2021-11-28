package frc.auton.guiauto.serialization;

import frc.auton.TemplateAuto;
import frc.utility.OrangeUtility;

public class Parser {
    //Again the TemplateAuto method argument is not necessary. We use it to access methods in our superclass of our auto
    public static boolean execute(String string, TemplateAuto context){
        String[] commands = string.split("\n"); //Start by splitting everything by lines (commands)
        for (String command : commands) {
            if(command.startsWith("#")) continue; //Ignore comments
            String[] parts = command.split(" "); //Split by spaces
            if(parts.length == 0) continue; //Ignore empty lines
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
                    //Add more cases for other robot functions
                    default:
                        return false; //Command does not exist
                }
            } catch (Exception e) {
                return false;
            }
        }
        return true;
    }
}