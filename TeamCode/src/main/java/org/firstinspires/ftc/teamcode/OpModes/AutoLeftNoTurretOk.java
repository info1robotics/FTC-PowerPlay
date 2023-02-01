package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.async;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.inline;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.pause;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.sync;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.trajectory;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.SubSystems.Claw;

@Autonomous(name = "Auto Left Cycles")
@Config
public class AutoLeftNoTurretOk extends AutoBase {
    public Trajectory spline_to_high, start_to_align, high_to_stack, stack_to_high;

    @Override
    public void onInit() {
        start_to_align = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-35, -23.5))
                .build();

        spline_to_high = drive.trajectoryBuilder(start_to_align.end())
                .splineTo(new Vector2d(-29.5, -8), Math.toRadians(45))
                .build();

        high_to_stack = drive.trajectoryBuilder(spline_to_high.end(), true)
                .splineTo(new Vector2d(-55,-8), Math.toRadians(180))
                .build();

        stack_to_high = drive.trajectoryBuilder(high_to_stack.end())
                .splineTo(new Vector2d(-32, -8.25), Math.toRadians(45))
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
                                    linkage.goToLevel(630, 0.3);
                                })
                        )
                ),
                pause(500),
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
                                pause(1000),
                                trajectory(high_to_stack)
                        ),
                        sync(
                                pause(300),
                                // positia de high X, inaltimea linkage
                                inline(() -> linkage.goToLevel(125,0.2)) //115
                        ),
                        sync(
                                inline(() -> {turret.disengageSuperBrake(); turret.disengageBrake();}),
                                pause(300),
                                inline(() -> turret.goToAngleAuto(170, 1.0)), //165 215
                                pause(1000),
                                inline(() -> {turret.engageSuperBrake(); turret.engageBrake();})
                        )
                ),
                pause(500),
                inline(() -> claw.setState(Claw.states.CLOSED)),
                pause(500),
                async(
                        inline(() -> linkage.setTargetPosition(350))
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
                                inline(() -> linkage.setTargetPosition(500)),
                                inline(() -> {turret.disengageSuperBrake(); turret.disengageBrake();}),
                                pause(100),
                                inline(() -> turret.goToAngleAuto(-5, 0.2)), //10
                                pause(1000),
                                inline(() -> {turret.engageSuperBrake(); turret.engageBrake();})
                        ),
                        sync(
                                pause(500),
                                inline(() -> linkage.goToLevel(630, 0.2))
                        )
                ),
                pause(1000),
                inline(() -> linkage.goToLevel(500, 0.3)),
                pause(400),
                inline(() -> claw.setState(Claw.states.OPEN)),
                pause(1000)
        );
    }
}
