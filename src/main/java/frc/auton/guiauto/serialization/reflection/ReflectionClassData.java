package frc.auton.guiauto.serialization.reflection;

import java.lang.reflect.Method;
import java.util.Arrays;

public final class ReflectionClassData {
    public final String fullName;
    public final String[] fieldNames;
    public final String[] fieldTypes;
    public final ReflectionMethodData[] methods;
    public final int modifiers;
    public final boolean isEnum;


    public ReflectionClassData(Class clazz) {
        this.fullName = clazz.getName();
        Method[] methods = clazz.getMethods();
        this.methods = new ReflectionMethodData[methods.length];
        for (int i = 0; i < methods.length; i++) {
            this.methods[i] = new ReflectionMethodData(methods[i]);
        }

        this.fieldNames = new String[clazz.getDeclaredFields().length];
        this.fieldTypes = new String[clazz.getDeclaredFields().length];
        for (int i = 0; i < clazz.getDeclaredFields().length; i++) {
            this.fieldNames[i] = clazz.getDeclaredFields()[i].getName();
            this.fieldTypes[i] = clazz.getDeclaredFields()[i].getType().getName();
        }

        this.isEnum = clazz.isEnum();

        modifiers = clazz.getModifiers();
    }

    @Override
    public String toString() {
        return "ReflectionClassData{" +
                "fullName='" + fullName + '\'' +
                ", fieldNames=" + Arrays.toString(fieldNames) +
                ", fieldTypes=" + Arrays.toString(fieldTypes) +
                ", methods=" + Arrays.toString(methods) +
                '}';
    }
}
