package org.firstinspires.ftc.teamcode.OpModes.Autonomous;


import static org.firstinspires.ftc.teamcode.SubSystems.Linkage.DESIRED_HEIGHT;
import static org.firstinspires.ftc.teamcode.SubSystems.Turret.AUTO_SPEED;
import static org.firstinspires.ftc.teamcode.SubSystems.Turret.DESIRED_ANGLE;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.async;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.inline;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.pause;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.sync;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.trajectory;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;

import java.util.Arrays;

//@Disabled
@Autonomous(name = "Left Park")
@Config
public class AutoParkLeft extends AutoBase {
    public Pose2d startPoseLeft;
    public Trajectory align_preload;
    public TrajectorySequence stack_to_low, preload_to_turn, turn_to_stack;
    public TrajectorySequence run_to_zone_3, run_to_zone_2, run_to_zone_1;

    TrajectoryVelocityConstraint slowConstraint = new MinVelocityConstraint(Arrays.asList(
            new TranslationalVelocityConstraint(40),
            new AngularVelocityConstraint(2)
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
        align_preload = drive.trajectoryBuilder(startPoseLeft)
                .lineTo(new Vector2d(startPoseLeft.getX()-1, -20))
                .build();

        preload_to_turn = drive.trajectorySequenceBuilder(align_preload.end())
                .lineTo(new Vector2d(startPoseLeft.getX()-1, -10))
                .build();

        turn_to_stack = drive.trajectorySequenceBuilder(preload_to_turn.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
            .turn(Math.toRadians(-100))
                .resetConstraints()
                .build();


        run_to_zone_3 = drive.trajectorySequenceBuilder(turn_to_stack.end())
                .setReversed(true)
                .lineTo(new Vector2d(-16.5, -8))
                .build();

        run_to_zone_2 = drive.trajectorySequenceBuilder(turn_to_stack.end())
                .setReversed(true)
                .lineTo(new Vector2d(-35, -8))
                .build();

        run_to_zone_1 = drive.trajectorySequenceBuilder(turn_to_stack.end())
                .setReversed(true)
                .lineTo(new Vector2d(-65, -8))
                .strafeRight(15)
                .build();

        task = sync(
                // Robot goes forward and aligns itself with the mid junction.
                inline(() -> claw.setState(Claw.states.CLOSED)),
                pause(200),
                inline(() -> DESIRED_HEIGHT = 400),
                async(
                trajectory(align_preload),
                inline(() -> {
                    turret.setDistanceThreshold(200);
                    DESIRED_ANGLE = 90;})
                ),
                inline(() -> turret.toggleSetThreshold(true)),
                sync(
                        pause(250),
                        inline(() -> DESIRED_HEIGHT = 250),
                        inline(() -> claw.setState(Claw.states.OPEN))
                ),
                inline(() -> turret.toggleSetThreshold(false)),
                trajectory(preload_to_turn),
                inline(() -> {
                    AUTO_SPEED = 0.3;
                    DESIRED_ANGLE = 0;
                }),
                trajectory(turn_to_stack),
                inline(() -> DESIRED_HEIGHT = -25),
                pause(18000),
                async(
                        inline(() -> {
                            if(x == 1) drive.followTrajectorySequence(run_to_zone_1);
                            else if (x == 3) drive.followTrajectorySequence(run_to_zone_3);
                        }),
                        inline(() -> {
                            AUTO_SPEED = 0.3;
                            DESIRED_ANGLE = 0;
                        }),
                        pause(200),
                        inline(() -> DESIRED_HEIGHT = 0)
                )
        );
    }
}
