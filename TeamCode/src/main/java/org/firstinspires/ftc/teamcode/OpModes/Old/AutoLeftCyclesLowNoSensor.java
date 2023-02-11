//package org.firstinspires.ftc.teamcode.OpModes;
//
//
//import static org.firstinspires.ftc.teamcode.SubSystems.Linkage.DESIRED_HEIGHT;
//import static org.firstinspires.ftc.teamcode.SubSystems.Turret.AUTO_SPEED;
//import static org.firstinspires.ftc.teamcode.SubSystems.Turret.DESIRED_ANGLE;
//import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.async;
//import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.inline;
//import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.pause;
//import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.sync;
//import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.trajectory;
//
//import com.acmerobotics.dashboard.config.Config;
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
//@Autonomous(name = "Left Cycles Low !Sensor")
//@Config
//public class AutoLeftCyclesLowNoSensor extends AutoBase {
//    public Trajectory align_preload;
//    public TrajectorySequence mid_preload, stack_to_low, preload_to_turn, low_to_stack, turn_to_stack, go_to_stack, low_to_stack_1, low_to_stack_2, low_to_stack_3, low_to_stack_4;
//    public TrajectorySequence cycle_high_1, cycle_high_2, cycle_high_3, cycle_high_4, cycle_high_5;
//    public TrajectorySequence cycle_stack_1, cycle_stack_2, cycle_stack_3, cycle_stack_4, cycle_stack_5;
//    public static double HIGH_TURRET_ANGLE = -130;
//    public static double PRELOAD_HIGH_X = -34, PRELOAD_HIGH_Y = -10;
//    public static double STACK_X = -59, STACK_Y = -9;
//    public static double LOW_X = -34, LOW_Y = -10;
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
//                .lineTo(new Vector2d(startPose.getX()-1, -20))
//                .build();
//
//        preload_to_turn = drive.trajectorySequenceBuilder(align_preload.end())
//                .lineTo(new Vector2d(startPose.getX()-1, -9))
//                .build();
//
//        turn_to_stack = drive.trajectorySequenceBuilder(preload_to_turn.end())
//            .turn(Math.toRadians(90))
//            .build();
//
//        go_to_stack = drive.trajectorySequenceBuilder(turn_to_stack.end())
//                .setAccelConstraint(accelConstraint)
//                .setVelConstraint(slowConstraint)
//                .lineTo(new Vector2d(-58, -9))
//                .resetConstraints()
//                .build();
//
//        stack_to_low = drive.trajectorySequenceBuilder(go_to_stack.end())
//                .setReversed(true)
//                .lineTo(new Vector2d(-50, -9))
//                .build();
//
//        low_to_stack = drive.trajectorySequenceBuilder(stack_to_low.end())
//                .lineTo(new Vector2d(-59, -9))
//                .build();
//
//        low_to_stack_1 = drive.trajectorySequenceBuilder(stack_to_low.end())
//                .lineTo(new Vector2d(-58.5, -9))
//                .build();
//
//        low_to_stack_2 = drive.trajectorySequenceBuilder(stack_to_low.end())
//                .lineTo(new Vector2d(-58, -9))
//                .build();
//
//        low_to_stack_3 = drive.trajectorySequenceBuilder(stack_to_low.end())
//                .lineTo(new Vector2d(-58, -9))
//                .build();
//
//        low_to_stack_4 = drive.trajectorySequenceBuilder(stack_to_low.end())
//                .lineTo(new Vector2d(-58, -9))
//                .build();
////
////        stack_to_low = drive.trajectorySequenceBuilder(high_to_stack.end())
////                .lineTo(new Vector2d(LOW_X, LOW_Y))
////                .build();
////
////        cycle_stack_1 = drive.trajectorySequenceBuilder(mid_preload.end())
////                .setReversed(true)
////                .splineTo(new Vector2d(-50,-9), Math.toRadians(180))
////                .setAccelConstraint(accelConstraint)
////                .setVelConstraint(slowConstraint)
////                .lineTo(new Vector2d(CYCLE1_STACK_X, CYCLE1_STACK_Y))
////                .resetConstraints()
////                .build();
////
////        cycle_high_1 = drive.trajectorySequenceBuilder(high_to_stack.end())
////                .lineTo(new Vector2d(-55, -8))
////                .setAccelConstraint(accelConstraint)
////                .setVelConstraint(slowConstraint)
////                .splineToSplineHeading(new Pose2d(CYCLE1_HIGH_X, CYCLE1_HIGH_Y, Math.toRadians(45)), Math.toRadians(0))
////                .resetConstraints()
////                .build();
//
//
//
//        task = sync(
//                // Robot goes forward and aligns itself with the mid junction.
//                inline(() -> claw.setState(Claw.states.CLOSED)),
//                pause(300),
//                inline(() -> DESIRED_HEIGHT = 400),
//                async(
//                trajectory(align_preload),
//                inline(() -> DESIRED_ANGLE = 90)
//                ),
//                sync(
//                        pause(1000),
//                        inline(() -> DESIRED_HEIGHT = 250),
//                        pause(100),
//                        inline(() -> claw.setState(Claw.states.OPEN))
//                ),
//                async(
//                        trajectory(preload_to_turn),
//                        sync(
//                                pause(100),
//                                inline(() -> DESIRED_HEIGHT = 80)
//                        )
//                ),
//
//                // first cycle
//
//                inline(() -> DESIRED_ANGLE = 0),
//                pause(300),
//                trajectory(go_to_stack),
//                pause(100),
//                inline(() -> claw.setState(Claw.states.CLOSED)),
//                inline(() -> DESIRED_HEIGHT = 150),
//                pause(200),
//                async(
//                        trajectory(stack_to_low),
//                        sync(
//                                pause(200),
//                                inline(() -> DESIRED_HEIGHT = 250),
//                                inline(() -> AUTO_SPEED = 0.25),
//                                inline(() -> {
//                                    DESIRED_ANGLE = HIGH_TURRET_ANGLE;
//                                    turret.setDistanceThreshold(160);
//                                }),
//                                pause(1000)
//                                )
//                ),
//                inline(() -> turret.toggleSetThreshold(true)),
//                pause(1000),
//                inline(() -> DESIRED_HEIGHT = 150),
//                pause(100),
//                inline(() -> claw.setState(Claw.states.OPEN)),
//                inline(() -> turret.toggleSetThreshold(false)),
//
//                // second cycle
//
//                inline(() -> DESIRED_ANGLE = 0),
//                pause(1000),
//                inline(() -> DESIRED_HEIGHT = 70),
//                pause(300),
//                trajectory(low_to_stack_1),
//                pause(100),
//                inline(() -> claw.setState(Claw.states.CLOSED)),
//                pause(100),
//                inline(() -> DESIRED_HEIGHT = 150),
//                async(
//                        trajectory(stack_to_low),
//                        sync(
//                                pause(200),
//                                inline(() -> DESIRED_HEIGHT = 250),
//                                inline(() -> AUTO_SPEED = 0.20),
//                                inline(() -> DESIRED_ANGLE = HIGH_TURRET_ANGLE),
//                                pause(500)
//                        )
//                ),
//                pause(2000),
//                inline(() -> DESIRED_HEIGHT = 150),
//                pause(100),
//                inline(() -> claw.setState(Claw.states.OPEN)),
//                // third cycle
//
//                inline(() -> DESIRED_ANGLE = 0),
//                pause(1000),
//                inline(() -> DESIRED_HEIGHT = 60),
//                pause(300),
//                trajectory(low_to_stack_2),
//                pause(100),
//                inline(() -> claw.setState(Claw.states.CLOSED)),
//                pause(200),
//                inline(() -> DESIRED_HEIGHT = 150),
//                pause(200),
//                async(
//                        trajectory(stack_to_low),
//                        sync(
//                                pause(200),
//                                inline(() -> DESIRED_HEIGHT = 250),
//                                inline(() -> AUTO_SPEED = 0.20),
//                                inline(() -> {
//                                    DESIRED_ANGLE = HIGH_TURRET_ANGLE;
//                                    turret.setDistanceThreshold(160);
//                                }),
//                                pause(1000)
//                        )
//                ),
//                inline(() -> turret.toggleSetThreshold(true)),
//                pause(1000),
//                inline(() -> DESIRED_HEIGHT = 150),
//                pause(100),
//                inline(() -> claw.setState(Claw.states.OPEN)),
//                inline(() -> turret.toggleSetThreshold(false)),
//
//                // fourth cycle
//
//                inline(() -> DESIRED_ANGLE = 0),
//                pause(1000),
//                inline(() -> DESIRED_HEIGHT = 60),
//                pause(300),
//                trajectory(low_to_stack_3),
//                pause(100),
//                inline(() -> claw.setState(Claw.states.CLOSED)),
//                pause(200),
//                inline(() -> DESIRED_HEIGHT = 150),
//                pause(100),
//                async(
//                        trajectory(stack_to_low),
//                        sync(
//                                pause(200),
//                                inline(() -> DESIRED_HEIGHT = 250),
//                                inline(() -> {
//                                    DESIRED_ANGLE = HIGH_TURRET_ANGLE;
//                                    turret.setDistanceThreshold(160);
//                                }),
//                                pause(1000)
//                        )
//                ),
//                inline(() -> turret.toggleSetThreshold(true)),
//                pause(1000),
//                inline(() -> DESIRED_HEIGHT = 150),
//                pause(100),
//                inline(() -> claw.setState(Claw.states.OPEN)),
//                inline(() -> turret.toggleSetThreshold(false)),
//
//                // third cycle
//
//                inline(() -> DESIRED_ANGLE = 0),
//                pause(1000),
//                inline(() -> DESIRED_HEIGHT = 25),
//                pause(300),
//                trajectory(low_to_stack_4),
//                pause(100),
//                inline(() -> claw.setState(Claw.states.CLOSED)),
//                pause(200),
//                inline(() -> DESIRED_HEIGHT = 150),
//                pause(200),
//                async(
//                        trajectory(stack_to_low),
//                        sync(
//                                pause(200),
//                                inline(() -> DESIRED_HEIGHT = 250),
//                                inline(() -> AUTO_SPEED = 0.20),
//                                inline(() -> {
//                                    DESIRED_ANGLE = HIGH_TURRET_ANGLE;
//                                    turret.setDistanceThreshold(160);
//                                }),
//                                pause(1000)
//                        )
//                ),
//                inline(() -> turret.toggleSetThreshold(true)),
//                pause(1000),
//                inline(() -> DESIRED_HEIGHT = 150),
//                pause(100),
//                inline(() -> claw.setState(Claw.states.OPEN)),
//                inline(() -> turret.toggleSetThreshold(false))
//
//                // First cycle begins here
////
////                async(
////                        sync(
////                                pause(750),
////                                trajectory(high_to_stack)
////                        ),
////                        sync(
////                                pause(100),
////                                inline(() -> DESIRED_HEIGHT = 100)
////                        ),
////                        sync(
////                                inline(() -> DESIRED_ANGLE = -180),
////                                pause(1000)
////                        )
////                ),
////
////                pause(500),
////
////                // Secure the cone
////                inline(() -> claw.setState(Claw.states.CLOSED)),
////                pause(500),
////                inline(() -> DESIRED_HEIGHT = 200),
////
////                async(
////                        trajectory(stack_to_low),
////                        sync(
////                                inline(() -> DESIRED_HEIGHT = 300),
////                                pause(100),
////                                inline(() -> DESIRED_ANGLE = -270),
////                                pause(1000)
////                        )
////                ),
////                inline(() -> DESIRED_HEIGHT = 400),
////                pause(500),
////                inline(() -> DESIRED_HEIGHT = 300),
////                pause(500),
////                inline(() -> claw.setState(Claw.states.OPEN)),
////                pause(500)
//        );
//    }
//}
