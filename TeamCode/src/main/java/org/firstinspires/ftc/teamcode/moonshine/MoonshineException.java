package org.firstinspires.ftc.teamcode.moonshine;

public class MoonshineException extends RuntimeException {
    public MoonshineException(Command command, String explanation) {
        super(getExplanationString(command, explanation));
    }

    public static String getExplanationString(Command command, String explanation) {
        // TODO: add command state to explanation
        return explanation;
    }
}
