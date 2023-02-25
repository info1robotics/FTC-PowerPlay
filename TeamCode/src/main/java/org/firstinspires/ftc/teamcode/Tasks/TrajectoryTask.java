package org.firstinspires.ftc.teamcode.Tasks;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

public class TrajectoryTask extends Task {
    Trajectory trajectory;
    TrajectorySequence trajectorySequence;

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

