package org.firstinspires.ftc.teamcode.Tasks;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;

public class TrajectorySequenceTask extends Task {

    TrajectorySequence trajectorySequence;

    public TrajectorySequenceTask(TrajectorySequence trajectory) {
        this.trajectorySequence = trajectory;
    }



    @Override
    public void tick() {
        if (!Thread.currentThread().isInterrupted() && context.drive.isBusy()) context.drive.update();
        else state = State.FINISHED;
    }

    @Override
    public void run() {
        context.drive.followTrajectorySequenceAsync(trajectorySequence);
    }
}

