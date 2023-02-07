//package org.firstinspires.ftc.teamcode.OpModes;
//
//import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.async;
//import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.inline;
//import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.pause;
//import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.sync;
//import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.trajectory;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//
//import org.firstinspires.ftc.teamcode.SubSystems.Claw;
//@Disabled
//@Autonomous(name = "Left Preload Old")
//@Config
//public class AutoLeftPreload extends AutoBase {
//    public Trajectory spline_to_high, start_to_align, high_to_stack, stack_to_high;
//
//    public static double HIGH_TO_STACK_X = -51.7;
//    public static double HIGH_TO_STACK_Y = -8.0;
//
//    @Override
//    public void onInit() {
//        start_to_align = drive.trajectoryBuilder(startPose)
//                .lineTo(new Vector2d(-35, -23.5))
//                .build();
//
//        spline_to_high = drive.trajectoryBuilder(start_to_align.end())
//                .splineTo(new Vector2d(-29.5, -8.25), Math.toRadians(45))
//                .build();
//
//        high_to_stack = drive.trajectoryBuilder(spline_to_high.end())
//                .lineTo(new Vector2d(HIGH_TO_STACK_X, HIGH_TO_STACK_Y))
//                .build();
//
//        stack_to_high = drive.trajectoryBuilder(high_to_stack.end())
//                .lineToLinearHeading(new Pose2d(-19.5, -14.0, Math.toRadians(0)))
//                .build();
//
//        task = sync(
//                inline(() -> claw.setState(Claw.states.CLOSED)),
//                pause(150),
//                inline(() -> {
//                    linkage.goToLevel(100, 0.3);
//                }),
//                async(
//                        trajectory(start_to_align),
//                        inline(() -> {
//                            linkage.goToLevel(250, 0.3);
//                        })
//                ),
//
//                async(
//                        trajectory(spline_to_high),
//                        sync(
//                                pause(300),
//                                inline(() -> {
//                                    linkage.goToLevel(630, 0.3);
//                                })
//                        )
//                ),
//                pause(500),
//                async(
//                        inline(() -> linkage.goToLevel(450, 0.3)),
//                        sync(
//                                pause(500),
//                                inline(() -> claw.setState(Claw.states.OPEN))
//                        )
//                ),
//                pause(300)
//
//        );
//    }
//}
