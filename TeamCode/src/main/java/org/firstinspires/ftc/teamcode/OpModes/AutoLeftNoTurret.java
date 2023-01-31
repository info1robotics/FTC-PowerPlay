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
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;

@Autonomous(name = "Auto Left No Turret")
@Config
public class AutoLeftNoTurret extends AutoBase {
    public Trajectory spline_to_high, start_to_align, high_to_stack, stack_to_high;
    public static double SPLINE_TO_HIGH_X = -29.4;
    public static double SPLINE_TO_HIGH_Y = -8.65;
    public static int SPLINE_ANGLE = 90;

    public static double HIGH_TO_STACK_X = -40.7;
    public static double HIGH_TO_STACK_Y = -10.0;
    public static double STACK_ANGLE = 180;
    public static double STACK_TURRET_SPEED = 0.3;

    @Override
    public void onInit() {
        start_to_align = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-35, -23.5))
                .build();

        spline_to_high = drive.trajectoryBuilder(start_to_align.end())
//                .setVelConstraint(30)
                .splineTo(new Vector2d(SPLINE_TO_HIGH_X, SPLINE_TO_HIGH_Y), Math.toRadians(45))
                .build();

        high_to_stack = drive.trajectoryBuilder(spline_to_high.end())
                .lineToConstantHeading(new Vector2d(HIGH_TO_STACK_X, HIGH_TO_STACK_Y))
                .build();

        stack_to_high = drive.trajectoryBuilder(high_to_stack.end())
                .lineToLinearHeading(new Pose2d(-19.5, -14.0, Math.toRadians(0)))
                .build();

        task = sync(
                inline(() -> claw.setState(Claw.states.CLOSED)),
                pause(150),
                inline(() -> {
                    linkage.goToLevel(100, 0.3);
                }),
                trajectory(start_to_align),
                pause(500),
                async (
                        trajectory(spline_to_high),
                        sync(
                                pause(200),
                                inline(() -> {
                                    turret.goToAngle(0, 1.0);
                                    linkage.goToLevel(600, 0.35);
                                })
                        )
                ),
                pause(1000),
                async(
                        inline(() -> linkage.goToLevel(450, 0.45)),
                        sync(
                                pause(300),
                                inline(() -> claw.setState(Claw.states.OPEN))
                        )
                ),
                pause(900),
                async(
                        sync(
                                pause(300),
                                inline(() -> {
//                                    turret.goToAngle(STACK_ANGLE, STACK_TURRET_SPEED);
                                    linkage.goToLevel(110, 0.3);
                                })
                        ),
                        trajectory(high_to_stack)
                )
//                pause(300),
//                inline(() -> claw.setState(Claw.states.CLOSED)),
//                pause(300),
//                async(
//                        inline(() -> {
//                            linkage.goToLevel(550, 0.3);
//                            turret.goToAngle(45, 1.0);
//                        }),
//                        trajectory(stack_to_high)
//                ),
//                pause(1000),
//                inline(() -> claw.setState(Claw.states.OPEN)),
//                pause(3000)
        );
    }
}
