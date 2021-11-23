package frc.auton.guiauto.serialization;

import frc.auton.TemplateAuto;
import frc.utility.OrangeUtility;

public class Parser {
    //Again the TemplateAuto method argument is not necessary. We use it to access methods in our superclass of our auto
    public static boolean execute(String string, TemplateAuto context){
        String[] commands = string.split("\n"); //Start by spliting everything by lines (commands)
        for (String command : commands) {
            StringBuilder methodName = null;
            StringBuilder argument = null;
            for (int i = 0; i < command.length(); i++) { //Loop though all the characters
                char character = command.charAt(i);
                if(argument != null){
                    //We've reached the argument part of the command. Everything else is part of the argument
                    argument.append(character);
                } else if(methodName == null){
                    //We haven't initalized the beginning of the methodName part of the command yet
                    //This means we haven't seen the first character of the method name yet
                    if(!Character.isWhitespace(character)){
                        //If we see the first character initalize the method name add add the character
                        methodName = new StringBuilder().append(character);
                    } //else do nothing and continue looping until we find our first character
                } else if(!Character.isWhitespace(character)){
                    // Keep adding to the method name until we hit whitespace
                    methodName.append(character);
                } else {
                    //We've hit section of whitespace directly before the argument part of the command
                    //Initalize the argument to record that we've reached this section and skip the first whitespace
                    argument = new StringBuilder();
                }
            }

            //System.out.println("Command " + command + " method name: " + methodName + " argument: " + argument);
            if(methodName != null){ //Check that we don't have a blank command/methodName
                try{    
                    //Figure out what the command is and execute it
                    switch (methodName.toString()){
                        case "print":
                            System.out.println(argument);
                            break;
                        case "sleep":
                                OrangeUtility.sleep(Long.parseLong(argument.toString()));
                            break;
                        default:
                            return false; 
                    }
                } catch (NumberFormatException | NullPointerException e) { return false; } 
            } //else and continue to the next command (or auto step)
            
        }
        return true;
    }
}