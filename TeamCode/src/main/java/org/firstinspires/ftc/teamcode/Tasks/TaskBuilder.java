package org.firstinspires.ftc.teamcode.Tasks;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;

public class TaskBuilder {
    public static SyncTask sync(Task... task) {
        return new SyncTask(task);
    }

    public static AsyncTask async(Task... task) {
        return new AsyncTask(task);
    }

    public static SleepTask pause(long ms) {
        return new SleepTask(ms);
    }

    public static TrajectoryTask trajectory(Trajectory trajectory) {
        return new TrajectoryTask(trajectory);
    }

    public static TrajectorySequenceTask trajectory(TrajectorySequence trajectory) {
        return new TrajectorySequenceTask(trajectory);
    }

    public static InlineTask inline(Runnable runnable) {
        return new InlineTask(runnable);
    }
}
