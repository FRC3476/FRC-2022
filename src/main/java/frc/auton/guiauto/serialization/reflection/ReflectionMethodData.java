package frc.auton.guiauto.serialization.reflection;

import org.jetbrains.annotations.NotNull;

import java.lang.reflect.Method;
import java.util.Arrays;


//Contains data about a method to be able  to error checking on it.
public final class ReflectionMethodData {

    public final @NotNull String methodName;
    public final String @NotNull [] parameterTypes;
    public final @NotNull String returnType;
    public final int modifiers;

    public ReflectionMethodData(@NotNull Method method) {
        this.methodName = method.getName();
        this.parameterTypes = ReflectionUtils.getParameterTypes(method);
        this.returnType = method.getReturnType().getTypeName();
        this.modifiers = method.getModifiers();
    }

    @Override
    public @NotNull String toString() {
        return "ReflectionMethodData{" +
                "methodName='" + methodName + '\'' +
                ", parameterTypes=" + Arrays.toString(parameterTypes) +
                ", returnType='" + returnType + '\'' +
                '}';
    }
}
