package frc.auton.guiauto.serialization.reflection;

import java.lang.reflect.Method;
import java.util.Arrays;


//Contains data about a method to be able  to error checking on it.
public final class ReflectionMethodData {

    public final String methodName;
    public final String[] parameterTypes;
    public final String returnType;
    public final int modifiers;

    public ReflectionMethodData(Method method) {
        this.methodName = method.getName();
        this.parameterTypes = ReflectionUtils.getParameterTypes(method);
        this.returnType = method.getReturnType().getTypeName();
        this.modifiers = method.getModifiers();
    }

    @Override
    public String toString() {
        return "ReflectionMethodData{" +
                "methodName='" + methodName + '\'' +
                ", parameterTypes=" + Arrays.toString(parameterTypes) +
                ", returnType='" + returnType + '\'' +
                '}';
    }
}
