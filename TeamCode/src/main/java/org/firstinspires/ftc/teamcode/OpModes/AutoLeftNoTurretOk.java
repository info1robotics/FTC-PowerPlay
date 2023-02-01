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

    public static double HIGH_TO_STACK_X = -51.7;
    public static double HIGH_TO_STACK_Y = -8.0;

    @Override
    public void onInit() {
        start_to_align = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-35, -23.5))
                .build();

        spline_to_high = drive.trajectoryBuilder(start_to_align.end())
                .splineTo(new Vector2d(-29.5, -8.25), Math.toRadians(45))
                .build();

        high_to_stack = drive.trajectoryBuilder(spline_to_high.end(), true)
                .splineTo(new Vector2d(-53.75,-9), Math.toRadians(180))
                .build();

        stack_to_high = drive.trajectoryBuilder(high_to_stack.end())
                .splineTo(new Vector2d(-29.5, -8.25), Math.toRadians(45))
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
                        trajectory(high_to_stack),
                        sync(
                                pause(300),
                                inline(() -> linkage.goToLevel(130,0.3))
                        ),
                        sync(
                                inline(() -> {turret.disengageSuperBrake(); turret.disengageBrake();}),
                                pause(100),
                                inline(() -> turret.goToAngleAuto(215, 0.75)),
                                pause(800),
                                inline(() -> {turret.engageSuperBrake(); turret.engageBrake();})
                        )
                ),
                pause(500),
                inline(() -> claw.setState(Claw.states.CLOSED)),
                pause(500),
                inline(() -> linkage.setTargetPosition(350)),
                pause(1000),
                async(
                        trajectory(stack_to_high),
                        sync(
                                inline(() -> {turret.disengageSuperBrake(); turret.disengageBrake();}),
                                pause(100),
                                inline(() -> turret.goToAngleAuto(20, 0.3)),
                                pause(800),
                                inline(() -> {turret.engageSuperBrake(); turret.engageBrake();})
                        ),
                        sync(
                                pause(800),
                                inline(() -> linkage.goToLevel(630, 0.3))
                        )
                ),
                inline(() -> claw.setState(Claw.states.OPEN)),
                pause(1000)
        );
    }
}
