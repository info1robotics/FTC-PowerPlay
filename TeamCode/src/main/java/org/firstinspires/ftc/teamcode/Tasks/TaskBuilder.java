package org.firstinspires.ftc.teamcode.Tasks;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

import java.util.function.Supplier;

public class TaskBuilder {
    public static SerialTask serial(Task... task) {
        return new SerialTask(task);
    }

    public static ParallelTask parallel(Task... task) {
        return new ParallelTask(task);
    }

    public static SleepTask sleepms(long ms) {
        return new SleepTask(ms);
    }

    public static TrajectoryTask trajectory(Trajectory trajectory) {
        return new TrajectoryTask(trajectory);
    }

    public static TrajectorySequenceTask trajectorySequence(TrajectorySequence trajectory) {
        return new TrajectorySequenceTask(trajectory);
    }

    public static ExecuteTask execute(Runnable runnable) {
        return new ExecuteTask(runnable);
    }
}
