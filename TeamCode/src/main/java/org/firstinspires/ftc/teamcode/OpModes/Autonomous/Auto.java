package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.geometry.*;
import com.acmerobotics.roadrunner.trajectory.constraints.*;

import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.SubSystems.V2.Claw;

import java.util.Arrays;

//@Disabled
@Autonomous(name = "Auto")
@Config
public class Auto extends AutoBase {
    public Pose2d startPoseLeft;
    public TrajectorySequence preload_high;

    TrajectoryVelocityConstraint slowConstraint = new MinVelocityConstraint(Arrays.asList(
            new TranslationalVelocityConstraint(40),
            new AngularVelocityConstraint(Math.toRadians(90))
    ));

    TrajectoryVelocityConstraint fastConstraint = new MinVelocityConstraint(Arrays.asList(
            new TranslationalVelocityConstraint(80)
    ));

    TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(20);
    TrajectoryAccelerationConstraint fastAccelConstraint = new ProfileAccelerationConstraint(80);

    @Override
    public void onInit() {
        startPoseLeft = new Pose2d(-37.5, -62, Math.toRadians(90));
        drive.setPoseEstimate(startPoseLeft);

//        align_preload = drive.trajectoryBuilder(startPoseLeft)
//                .lineTo(new Vector2d(startPoseLeft.getX()-1, -20))
//                .build();

        task = serial(
                execute(() -> claw.setClawState(Claw.clawStates.CLOSED))
        );
    }
}
