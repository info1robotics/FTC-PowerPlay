package org.firstinspires.ftc.teamcode.Tasks;

import java.util.Arrays;
import java.util.stream.Stream;

public class CompoundTask extends Task {
    public Task[] children;

    public CompoundTask(Task... tasks) {
        children = tasks;
    }

    public void add(Task task) {
        children = (Task[]) Stream.concat(Arrays.stream(children), Arrays.stream(new Task[]{task})).toArray();
    }
}
