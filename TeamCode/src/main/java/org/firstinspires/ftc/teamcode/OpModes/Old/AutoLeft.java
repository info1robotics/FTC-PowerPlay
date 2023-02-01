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
//
//import org.firstinspires.ftc.teamcode.SubSystems.Claw;
//
//@Autonomous(name = "Auto Left")
//@Config
//public class AutoLeft extends AutoBase {
//    public Trajectory start_to_align, spline_to_high, high_to_stack, stack_to_high;
//    public static double SPLINE_TO_HIGH_X = -21.0;
//    public static double SPLINE_TO_HIGH_Y = -15.0;
//    public static int SPLINE_ANGLE = 90;
//
//    public static double HIGH_TO_STACK_X = -51.7;
//    public static double HIGH_TO_STACK_Y = -8.0;
//    public static double STACK_ANGLE = 180;
//
//    @Override
//    public void onInit() {
//        start_to_align = drive.trajectoryBuilder(startPose)
//                .lineTo(new Vector2d(-35, -23.5))
//                .build();
//
//        spline_to_high = drive.trajectoryBuilder(start_to_align.end())
//                .splineTo(new Vector2d(SPLINE_TO_HIGH_X, SPLINE_TO_HIGH_Y), Math.toRadians(0))
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
//                trajectory(start_to_align),
//                async(
//                        inline(() -> {
//                            linkage.goToLevel(550, 0.35);
//                            turret.goToAngle(SPLINE_ANGLE, 1.0);
//                        }),
//                        trajectory(spline_to_high)
//                ),
//                pause(250),
//                async(
//                        inline(() -> linkage.goToLevel(500, 0.3)),
//                        sync(
//                                pause(1000),
//                                inline(() -> claw.setState(Claw.states.OPEN))
//                        )
//                ),
//                pause(300),
//                async(
//                        inline(() -> {
//                            turret.goToAngle(STACK_ANGLE, 1.0);
//                            linkage.goToLevel(110, 0.3);
//                        }),
//                        trajectory(high_to_stack)
//                ),
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
//        );
//    }
//}
