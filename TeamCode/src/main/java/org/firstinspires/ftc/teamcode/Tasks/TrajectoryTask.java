package org.firstinspires.ftc.teamcode.Tasks;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

public class TrajectoryTask extends Task {
    Trajectory trajectory;

    public TrajectoryTask(Trajectory trajectory) {
        this.trajectory = trajectory;
    }

    @Override
    public void tick() {
        if (!Thread.currentThread().isInterrupted() && context.drive.isBusy()) context.drive.update();
        else state = State.FINISHED;
    }

    @Override
    public void run() {
        context.drive.followTrajectoryAsync(trajectory);
    }
}

