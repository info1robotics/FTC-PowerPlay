package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.async;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.inline;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.pause;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.sync;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.trajectory;

import android.view.animation.AccelerateInterpolator;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.Tasks.TrajectorySequenceTask;

import java.util.Arrays;

@Autonomous(name = "Auto Left Cycles")
@Config
public class AutoLeftNoTurretOk extends AutoBase {
    public Trajectory spline_to_high, start_to_align, high_to_stack;
    public TrajectorySequence stack_to_high;
//    TrajectoryVelocityConstraint slowConstraint = new MinVelocityConstraint(Arrays.asList(
//            new TranslationalVelocityConstraint(20)
//    ));
//    TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(10);
    @Override
    public void onInit() {
        start_to_align = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-35, -23.5))
                .build();

        spline_to_high = drive.trajectoryBuilder(start_to_align.end())
                .splineTo(new Vector2d(-29.5, -8), Math.toRadians(45))
                .build();

        high_to_stack = drive.trajectoryBuilder(spline_to_high.end(), true)
                .splineTo(new Vector2d(-54.5,-8), Math.toRadians(180))
                .build();

        stack_to_high = drive.trajectorySequenceBuilder(high_to_stack.end())
//                .setAccelConstraint(accelConstraint)
//                .setVelConstraint(slowConstraint)
                .splineTo(new Vector2d(-31.5, -5), Math.toRadians(45))
                .build();

        task = sync(
                inline(() -> claw.setState(Claw.states.CLOSED)),
                pause(150),
                inline(() -> {
                    linkage.goToLevel(100, 0.3);
                }),
                async(
                        trajectory(start_to_align),
                        inline(() -> {
                            linkage.goToLevel(250, 0.3);
                        })
                ),

                async(
                        trajectory(spline_to_high),
                        sync(
                                pause(300),
                                inline(() -> {
                                    linkage.goToLevel(650, 0.3);
                                })
                        )
                ),
                pause(100),
                async(
                        inline(() -> linkage.goToLevel(450, 0.3)),
                        sync(
                                pause(500),
                                inline(() -> claw.setState(Claw.states.OPEN))
                        )
                ),
                pause(300),
                async(
                        sync(
                                pause(500),
                                trajectory(high_to_stack)
                        ),
                        sync(
                                pause(100),
                                // positia de high X, inaltimea linkage
                                inline(() -> linkage.goToLevel(120,0.3)) //fluctuates
                        ),
                        sync(
                                inline(() -> {turret.disengageSuperBrake(); turret.disengageBrake();}),
                                pause(300),
                                inline(() -> turret.goToAngleAuto(175, 0.5)), //165 215
                                pause(1000),
                                inline(() -> {turret.engageSuperBrake(); turret.engageBrake();})
                        )
                ),
                pause(500),
                inline(() -> claw.setState(Claw.states.CLOSED)),
                pause(500),
                async(
                        inline(() -> linkage.goToLevel(450, 0.2))
//                        sync(
//                                inline(() -> {turret.disengageSuperBrake(); turret.disengageBrake();}),
//                                pause(100),
//                                inline(() -> turret.goToAngleAuto(90, 0.2)),
//                                pause(500),
//                                inline(() -> {turret.engageSuperBrake(); turret.engageBrake();})
//                                )
                        ),
                pause(1000),
                async(
                        trajectory(stack_to_high),
                        sync(
                                inline(() -> linkage.setTargetPosition(400)),
                                inline(() -> {turret.disengageSuperBrake(); turret.disengageBrake();}),
                                pause(100),
                                inline(() -> turret.goToAngleAuto(0, .3)), //10
                                pause(1000),
                                inline(() -> {turret.engageSuperBrake(); turret.engageBrake();})
                        )
//                        sync(
//                                pause(500)
//                        )
                ),
//                pause(1000),
                inline(() -> linkage.goToLevel(650, 0.3)),
                pause(500),
                inline(() -> linkage.goToLevel(500, 0.3)),
                pause(500),
                inline(() -> claw.setState(Claw.states.OPEN)),
                pause(500),
                async(
                        sync(
                                pause(500),
                                trajectory(high_to_stack)
                        ),
                        sync(
                                pause(100),
                                // positia de high X, inaltimea linkage
                                inline(() -> linkage.goToLevel(100,0.3)) //fluctuates
                        ),
                        sync(
                                inline(() -> {turret.disengageSuperBrake(); turret.disengageBrake();}),
                                pause(300),
                                inline(() -> turret.goToAngleAuto(175, 0.5)), //165 215
                                pause(1000),
                                inline(() -> {turret.engageSuperBrake(); turret.engageBrake();})
                        )
                ),
                pause(500),
                inline(() -> claw.setState(Claw.states.CLOSED)),
                pause(500),
                async(
                        inline(() -> linkage.goToLevel(450, 0.2))
//                        sync(
//                                inline(() -> {turret.disengageSuperBrake(); turret.disengageBrake();}),
//                                pause(100),
//                                inline(() -> turret.goToAngleAuto(90, 0.2)),
//                                pause(500),
//                                inline(() -> {turret.engageSuperBrake(); turret.engageBrake();})
//                                )
                ),
                pause(1000),
                async(
                        trajectory(stack_to_high),
                        sync(
                                inline(() -> linkage.setTargetPosition(400)),
                                inline(() -> {turret.disengageSuperBrake(); turret.disengageBrake();}),
                                pause(100),
                                inline(() -> turret.goToAngleAuto(0, .3)), //10
                                pause(1000),
                                inline(() -> {turret.engageSuperBrake(); turret.engageBrake();})
                        )),
                        inline(() -> linkage.goToLevel(650, 0.3)),
                        pause(500),
                        inline(() -> linkage.goToLevel(500, 0.3)),
                        pause(500),
                        inline(() -> claw.setState(Claw.states.OPEN)),
                        pause(500)
//                        sync(
//                                pause(500)
//                        )
        );
    }
}
