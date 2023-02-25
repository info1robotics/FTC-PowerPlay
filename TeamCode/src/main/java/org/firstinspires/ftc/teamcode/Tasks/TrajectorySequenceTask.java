package org.firstinspires.ftc.teamcode.Tasks;

import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

public class TrajectorySequenceTask extends Task {

    TrajectorySequence trajectorySequence;

    public TrajectorySequenceTask(TrajectorySequence trajectorySequence) {
        this.trajectorySequence = trajectorySequence;
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

