//package org.firstinspires.ftc.teamcode.OpModes;
//
//
//import static org.firstinspires.ftc.teamcode.SubSystems.Turret.DESIRED_ANGLE;
//import static org.firstinspires.ftc.teamcode.SubSystems.Linkage.DESIRED_HEIGHT;
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
//import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//
//import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.SubSystems.Claw;
//
//import java.util.Arrays;
//
//@Disabled
//@Autonomous(name = "Left Cycles Upgraded")
//@Config
//public class AutoLeftCyclesSequenced extends AutoBase {
//    public Trajectory align_preload;
//    public TrajectorySequence high_preload, stack_to_high, high_to_stack;
//    public TrajectorySequence cycle_high_1, cycle_high_2, cycle_high_3, cycle_high_4, cycle_high_5;
//    public TrajectorySequence cycle_stack_1, cycle_stack_2, cycle_stack_3, cycle_stack_4, cycle_stack_5;
//
//    public static double PRELOAD_HIGH_X = -34, PRELOAD_HIGH_Y = -10;
//    public static double STACK_X = -59, STACK_Y = -9;
//    public static double HIGH_X = -34, HIGH_Y = -10;
//
//    public static double CYCLE1_HIGH_X = 0.0, CYCLE1_HIGH_Y = 0.0, CYCLE1_STACK_X = 0.0, CYCLE1_STACK_Y = 0.0;
//    public static double CYCLE2_HIGH_X = 0.0, CYCLE2_HIGH_Y = 0.0, CYCLE2_STACK_X = 0.0, CYCLE2_STACK_Y = 0.0;
//    public static double CYCLE3_HIGH_X = 0.0, CYCLE3_HIGH_Y = 0.0, CYCLE3_STACK_X = 0.0, CYCLE3_STACK_Y = 0.0;
//    public static double CYCLE4_HIGH_X = 0.0, CYCLE4_HIGH_Y = 0.0, CYCLE4_STACK_X = 0.0, CYCLE4_STACK_Y = 0.0;
//    public static double CYCLE5_HIGH_X = 0.0, CYCLE5_HIGH_Y = 0.0, CYCLE5_STACK_X = 0.0, CYCLE5_STACK_Y = 0.0;
//
//    TrajectoryVelocityConstraint slowConstraint = new MinVelocityConstraint(Arrays.asList(
//            new TranslationalVelocityConstraint(40),
//            new AngularVelocityConstraint(3)
//    ));
//    TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(20);
//
//
//    @Override
//    public void onInit() {
//        align_preload = drive.trajectoryBuilder(startPose)
//                .lineTo(new Vector2d(-35, -23.5))
//                .build();
//
//        high_preload = drive.trajectorySequenceBuilder(align_preload.end())
//                .setAccelConstraint(accelConstraint)
//                .setVelConstraint(slowConstraint)
//                .splineTo(new Vector2d(PRELOAD_HIGH_X, PRELOAD_HIGH_Y), Math.toRadians(45))
//                .resetConstraints()
//                .build();
//
//        high_to_stack = drive.trajectorySequenceBuilder(high_preload.end())
//                .setReversed(true)
//                .setAccelConstraint(accelConstraint)
//                .setVelConstraint(slowConstraint)
////                .splineTo(new Vector2d(-50,-9), Math.toRadians(180))
//                .splineTo(new Vector2d(STACK_X,STACK_Y), Math.toRadians(180))
////                .lineTo(new Vector2d(STACK_X, STACK_Y))
//                .resetConstraints()
//                .build();
//
//        stack_to_high = drive.trajectorySequenceBuilder(high_to_stack.end())
//                .lineTo(new Vector2d(-55, -8))
//                .setAccelConstraint(accelConstraint)
//                .setVelConstraint(slowConstraint)
//                .splineToSplineHeading(new Pose2d(HIGH_X, HIGH_Y, Math.toRadians(45)), Math.toRadians(0))
//                .resetConstraints()
//                .build();
//
//        cycle_stack_1 = drive.trajectorySequenceBuilder(high_preload.end())
//                .setReversed(true)
//                .splineTo(new Vector2d(-50,-9), Math.toRadians(180))
//                .setAccelConstraint(accelConstraint)
//                .setVelConstraint(slowConstraint)
//                .lineTo(new Vector2d(CYCLE1_STACK_X, CYCLE1_STACK_Y))
//                .resetConstraints()
//                .build();
//
//        cycle_high_1 = drive.trajectorySequenceBuilder(high_to_stack.end())
//                .lineTo(new Vector2d(-55, -8))
//                .setAccelConstraint(accelConstraint)
//                .setVelConstraint(slowConstraint)
//                .splineToSplineHeading(new Pose2d(CYCLE1_HIGH_X, CYCLE1_HIGH_Y, Math.toRadians(45)), Math.toRadians(0))
//                .resetConstraints()
//                .build();
//
//
//
//        task = sync(
//                // Robot goes forward and aligns itself with the mid junction.
//                inline(() -> claw.setState(Claw.states.CLOSED)),
//                pause(300),
//                inline(() -> DESIRED_HEIGHT = 300),
//                trajectory(align_preload),
//
//                // The robot does a lil spline to align itself with the high junction.
//                async(
//                        trajectory(high_preload),
//                        inline(() -> DESIRED_HEIGHT = 630)
//                ),
//
//                // Drop the linkage a little to lock the junction
//                async(
//                        inline(() -> DESIRED_HEIGHT = 450),
//                        sync(
//                                pause(500),
//                                inline(() -> claw.setState(Claw.states.OPEN))
//                        )
//                ),
//
//                // First cycle begins here
//
//                async(
//                        sync(
//                                pause(750),
//                                trajectory(high_to_stack)
//                        ),
//                        sync(
//                                pause(100),
//                                inline(() -> DESIRED_HEIGHT = 100)
//                        ),
//                        sync(
//                                inline(() -> DESIRED_ANGLE = -180),
//                                pause(1000)
//                        )
//                ),
//                pause(500),
//
//                // Secure the cone
//                inline(() -> claw.setState(Claw.states.CLOSED)),
//                pause(500),
//                inline(() -> DESIRED_HEIGHT = 300),
//                async(
//                        trajectory(stack_to_high),
//                        sync(
//                                inline(() -> DESIRED_HEIGHT = 400),
//                                pause(100),
//                                inline(() -> DESIRED_ANGLE = 0),
//                                pause(1000)
//                        )
//                ),
//                inline(() -> DESIRED_HEIGHT = 630),
//                pause(500),
//                inline(() -> DESIRED_HEIGHT = 500),
//                pause(500),
//                inline(() -> claw.setState(Claw.states.OPEN)),
//                pause(500),
//
//                // Second cycle begins here
//
//                async(
//                sync(
//                        pause(750),
//                        trajectory(high_to_stack)
//                ),
//                sync(
//                        pause(100),
//                        inline(() -> DESIRED_HEIGHT = 90)
//                ),
//                sync(
//                        inline(() -> DESIRED_ANGLE = -180),
//                        pause(1000)
//                )),
//                pause(500),
//
//                // Secure the cone
//                inline(() -> claw.setState(Claw.states.CLOSED)),
//                pause(500),
//                inline(() ->  DESIRED_HEIGHT = 300),
//                pause(500),
//                async(
//                        trajectory(stack_to_high),
//                        sync(
//                                inline(() ->  DESIRED_HEIGHT = 400),
//                                pause(100),
//                                inline(() -> DESIRED_ANGLE = 0),
//                                pause(1000)
//                        )
//                ),
//                inline(() ->  DESIRED_HEIGHT = 630),
//                pause(500),
//                inline(() ->  DESIRED_HEIGHT = 500),
//                pause(500),
//                inline(() -> claw.setState(Claw.states.OPEN)),
//                pause(500),
//
//                // Third Cycle Begins Here
//
//                async(
//                        sync(
//                                pause(750),
//                                trajectory(high_to_stack)
//                        ),
//                        sync(
//                                pause(100),
//                                inline(() -> DESIRED_HEIGHT = 80)
//                        ),
//                        sync(
//                                inline(() -> DESIRED_ANGLE = -180),
//                                pause(1000)
//                        )
//                ),
//                pause(500),
//
//                // Secure the cone
//                inline(() -> claw.setState(Claw.states.CLOSED)),
//                pause(500),
//                inline(() -> DESIRED_HEIGHT = 300),
//                pause(500),
//                async(
//                        trajectory(stack_to_high),
//                        sync(
//                                inline(() -> DESIRED_HEIGHT = 400),
//                                pause(100),
//                                inline(() -> DESIRED_ANGLE = 0),
//                                pause(1000)
//                        )
//                ),
//                inline(() -> DESIRED_HEIGHT = 630),
//                pause(500),
//                inline(() -> DESIRED_HEIGHT = 500),
//                pause(500),
//                inline(() -> claw.setState(Claw.states.OPEN)),
//                pause(500)
//
//        );
//    }
//}
