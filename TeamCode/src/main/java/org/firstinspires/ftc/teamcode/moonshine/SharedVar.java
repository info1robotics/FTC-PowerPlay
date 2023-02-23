package org.firstinspires.ftc.teamcode.moonshine;

import java.util.function.Supplier;

public class SharedVar<T> {
    private final String name;
    private T defaultValue = null;

    public SharedVar(String name) {
        this.name = name;
    }

    public SharedVar(String name, T defaultValue) {
        this.name = name;
        this.defaultValue = defaultValue;
    }

    public T getValue() {
        if(!CommandEnv.getInstance().sharedVars.containsKey(name) && defaultValue != null)
            this.setValue(defaultValue);

        if(CommandEnv.getInstance().sharedVars.containsKey(name))
            return (T) CommandEnv.getInstance().sharedVars.get(name);
        else
            return null;
    }

    public Supplier<T> getSupplier() {
        return this::getValue;
    }

    public void setValue(T value) {
        CommandEnv.getInstance().sharedVars.put(name, value);
    }
}
