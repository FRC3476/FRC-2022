package frc.auton.guiauto.serialization.reflection;

import edu.wpi.first.wpilibj.Filesystem;
import frc.utility.Serializer;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Method;
import java.net.URL;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.List;

import static frc.auton.guiauto.serialization.reflection.ReflectionUtils.findClasses;

public final class ClassInformationSender {
    public ClassInformationSender() {

    }

    public static void updateReflectionInformation() {
        try {
            List<Class<?>> classes = findClasses(new File(Filesystem.getLaunchDirectory() + "/bin/main"), "");
            ArrayList<ReflectionClassData> reflectionClassData = new ArrayList<>();
            for (Class<?> aClass : classes) {
                reflectionClassData.add(new ReflectionClassData(aClass));
            }
            System.out.println(Serializer.serializeToString(reflectionClassData));

        } catch (ClassNotFoundException | IOException e) {
            e.printStackTrace();
        }
    }


}
