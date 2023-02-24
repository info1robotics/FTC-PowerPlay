package org.firstinspires.ftc.teamcode.moonshine;

public class SharedSubsystem<T extends Subsystem> extends SharedVar<T> {
    private Class<T> t;

    public SharedSubsystem(Class<T> T) {
        super(T.getName());
        t = T;
    }
    public SharedSubsystem(Class<T> T, T defaultValue) {
        super(T.getName(), defaultValue);
    }

    @Override
    public T getValue() throws MoonshineException {
        try {
            return super.getValue();
        } catch (MoonshineException e) {
            throw new MoonshineException(null, String.format("Subsystem of type %s is not initialized", t.getName()));
        }
    }
}
