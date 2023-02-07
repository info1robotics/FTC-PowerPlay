package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.async;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.inline;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.pause;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.sync;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.trajectory;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.SubSystems.Claw;
//@Disabled
@Autonomous(name = "Left Cycles")
@Config
public class AutoLeftCycles extends AutoBase {
    public Trajectory spline_to_high, start_to_align, high_to_stack, stack_to_high;

    // Velocity Constraints (just in case)
//    public TrajectorySequence stack_to_high;
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

        stack_to_high = drive.trajectoryBuilder(high_to_stack.end())
                // Constraints (works only in sequences)
//                .setAccelConstraint(accelConstraint)
//                .setVelConstraint(slowConstraint)
                .splineTo(new Vector2d(-31.5, -5), Math.toRadians(45))
                .build();

        task = sync(

                // Robot goes forward and aligns itself with the mid junction.
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

                // The robot does a spline to align itself with the high junction.
                async(
                        trajectory(spline_to_high),
                        sync(
                                pause(300),
                                inline(() -> {
                                    linkage.goToLevel(600, 0.7);
                                })
                        )
                ),

                // Drop the linkage a little to lock the junction
                pause(400),
                async(
                        inline(() -> linkage.goToLevel(450, 0.3)),
                        sync(
                                pause(1000),
                                inline(() -> claw.setState(Claw.states.OPEN))
                        )
                ),
                pause(300),

                // First cycle begins here
                async(
                        sync(
                                pause(500),
                                trajectory(high_to_stack)
                        ),
                        sync(
                                pause(100),
                                inline(() -> linkage.goToLevel(120,0.3))
                        ),
                        sync(
                                inline(() -> {turret.disengageSuperBrake(); turret.disengageBrake();}),
                                pause(300),
                                inline(() -> turret.goToAngleAuto(-180, 1.0)),
                                pause(1000)
//                                inline(() -> {turret.engageSuperBrake(); turret.engageBrake();})
                        )
                ),
                pause(500),

                // Secure the cone
                inline(() -> claw.setState(Claw.states.CLOSED)),
                pause(500),
                inline(() -> linkage.goToLevel(450, 0.2)),
                pause(500),
                async(
                        trajectory(stack_to_high),
                        sync(
                                inline(() -> linkage.setTargetPosition(400)),
                                inline(() -> {turret.disengageSuperBrake(); turret.disengageBrake();}),
                                pause(100),
                                inline(() -> turret.goToAngleAuto(0, 1.0)), //10
                                pause(1000)
//                                inline(() -> {turret.engageSuperBrake(); turret.engageBrake();})
                        )
                ),
                inline(() -> linkage.goToLevel(550, 0.3)),
                pause(500),
                inline(() -> linkage.goToLevel(500, 0.3)),
                pause(500),
                inline(() -> claw.setState(Claw.states.OPEN)),
                pause(500)

                // Second Cycle Begins Here
//
//                async(
//                        sync(
//                                pause(500),
//                                trajectory(high_to_stack)
//                        ),
//                        sync(
//                                pause(100),
//                                inline(() -> linkage.goToLevel(100,0.3))
//                        ),
//                        sync(
//                                inline(() -> {turret.disengageSuperBrake(); turret.disengageBrake();}),
//                                pause(300),
//                                inline(() -> turret.goToAngleAuto(175, 0.5)),
//                                pause(1000)
//                                //                                inline(() -> {turret.engageSuperBrake(); turret.engageBrake();})
//                        )
//                ),
//                pause(500),
//                inline(() -> claw.setState(Claw.states.CLOSED)),
//                pause(500),
//                inline(() -> linkage.goToLevel(450, 0.2)),
//                pause(500),
//                async(
//                        trajectory(stack_to_high),
//                        sync(
//                                inline(() -> linkage.setTargetPosition(400)),
//                                inline(() -> {turret.disengageSuperBrake(); turret.disengageBrake();}),
//                                pause(100),
//                                inline(() -> turret.goToAngleAuto(0, .3)),
//                                pause(1000)
////                                inline(() -> {turret.engageSuperBrake(); turret.engageBrake();})
//                        )),
//                        inline(() -> linkage.goToLevel(600, 0.7)),
//                        pause(500),
//                        inline(() -> linkage.goToLevel(500, 0.3)),
//                        pause(500),
//                        inline(() -> claw.setState(Claw.states.OPEN)),
//                        pause(500),
//
//                // Third Cycle Begins Here
//
//                async(
//                        sync(
//                                pause(500),
//                                trajectory(high_to_stack)
//                        ),
//                        sync(
//                                pause(100),
//                                inline(() -> linkage.goToLevel(100,0.3))
//                        ),
//                        sync(
//                                inline(() -> {turret.disengageSuperBrake(); turret.disengageBrake();}),
//                                pause(300),
//                                inline(() -> turret.goToAngleAuto(175, 0.5)),
//                                pause(1000),
//                                inline(() -> {turret.engageSuperBrake(); turret.engageBrake();})
//                        )
//                ),
//                pause(500),
//                inline(() -> claw.setState(Claw.states.CLOSED)),
//                pause(500),
//                inline(() -> linkage.goToLevel(450, 0.2)),
//                pause(500),
//                async(
//                        trajectory(stack_to_high),
//                        sync(
//                                inline(() -> linkage.setTargetPosition(400)),
//                                inline(() -> {turret.disengageSuperBrake(); turret.disengageBrake();}),
//                                pause(100),
//                                inline(() -> turret.goToAngleAuto(0, .3)),
//                                pause(700)
////                                inline(() -> {turret.engageSuperBrake(); turret.engageBrake();})
//                        )),
//                inline(() -> linkage.goToLevel(600, 0.7)),
//                pause(500),
//                inline(() -> linkage.goToLevel(500, 0.3)),
//                pause(300),
//                inline(() -> claw.setState(Claw.states.OPEN)),
//                pause(300)
        );
    }
}
