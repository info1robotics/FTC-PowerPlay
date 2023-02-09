package org.firstinspires.ftc.teamcode.OpModes;


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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;

import java.util.Arrays;

//@Disabled
@Autonomous(name = "Left Cycles")
@Config
public class AutoLeftCycles extends AutoBase {
    public Trajectory start_to_align, high_to_stack;

    // Velocity Constraints (just in case)
    public TrajectorySequence spline_to_high, stack_to_high;
    TrajectoryVelocityConstraint slowConstraint = new MinVelocityConstraint(Arrays.asList(
            new TranslationalVelocityConstraint(40),
            new AngularVelocityConstraint(3)
    ));
    TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(20);


    @Override
    public void onInit() {
        start_to_align = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-35, -23.5))
                .build();

        spline_to_high = drive.trajectorySequenceBuilder(start_to_align.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .splineTo(new Vector2d(-34, -10), Math.toRadians(45))
                .resetConstraints()
                .build();

        high_to_stack = drive.trajectoryBuilder(spline_to_high.end(), true)
                .splineTo(new Vector2d(-59,-9), Math.toRadians(180))
                .build();

        stack_to_high = drive.trajectorySequenceBuilder(high_to_stack.end())
                .lineTo(new Vector2d(-55, -8))
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .splineToSplineHeading(new Pose2d(-34, -10, Math.toRadians(45)), Math.toRadians(0))
                .resetConstraints()
                .build();

        task = sync(
//                inline(() -> linkage.goToLevel(250, 0.3)),
//                pause(500),
//                inline(() -> DESIRED_ANGLE = 90),
//                pause(5000),
//                inline(() -> DESIRED_ANGLE = 180),
//                pause(5000),
//                inline(() -> DESIRED_ANGLE = 0),
//                pause(5000),
//                inline(() -> DESIRED_ANGLE = -90),
//                pause(5000),
//                inline(() -> DESIRED_ANGLE = -180),
//                pause(10000)



                // Robot goes forward and aligns itself with the mid junction.
                inline(() -> claw.setState(Claw.states.CLOSED)),
                pause(300),
                inline(() -> {
                    linkage.goToLevel(300, 0.3);
                }),
                async(
                        trajectory(start_to_align),
                        inline(() -> {
                            linkage.goToLevel(300, 0.2);
                        })
                ),

                // The robot does a spline to align itself with the high junction.
                async(
                        trajectory(spline_to_high),
                        inline(() -> {
                            linkage.goToLevel(630, 0.15);
                        })
//                        sync(
//                                pause(300),
//                                inline(() -> {
//                                    linkage.goToLevel(600, 0.2);
//                                })
//                        )
                ),

                // Drop the linkage a little to lock the junction
                async(
                        inline(() -> linkage.goToLevel(450, 0.3)),
                        sync(
                                pause(500),
                                inline(() -> claw.setState(Claw.states.OPEN))
                        )
                ),

                // First cycle begins here

                async(
                        sync(
                                pause(750),
                                trajectory(high_to_stack)
                        ),
                        sync(
                                pause(100),
                                inline(() -> linkage.goToLevel(100,0.3))
                        ),
                        sync(
                                inline(() -> DESIRED_ANGLE = -180),
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
//                                inline(() -> {turret.disengageSuperBrake(); turret.disengageBrake();}),
                                pause(100),
                                inline(() -> DESIRED_ANGLE = 0),
                                pause(1000)
//                                inline(() -> {turret.engageSuperBrake(); turret.engageBrake();})
                        )
                ),
                inline(() -> linkage.goToLevel(630, 0.3)),
                pause(2000),
                inline(() -> linkage.goToLevel(500, 0.3)),
                pause(1000),
                inline(() -> claw.setState(Claw.states.OPEN)),
                pause(500),

        // second penis

        async(
                sync(
                        pause(750),
                        trajectory(high_to_stack)
                ),
                sync(
                        pause(100),
                        inline(() -> linkage.goToLevel(90,0.3))
                ),
                sync(
                        inline(() -> DESIRED_ANGLE = -180),
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
//                                inline(() -> {turret.disengageSuperBrake(); turret.disengageBrake();}),
                                pause(100),
                                inline(() -> DESIRED_ANGLE = 0),
                                pause(1000)
//                                inline(() -> {turret.engageSuperBrake(); turret.engageBrake();})
                        )
                ),
                inline(() -> linkage.goToLevel(630, 0.3)),
                pause(2000),
                inline(() -> linkage.goToLevel(500, 0.3)),
                pause(1000),
                inline(() -> claw.setState(Claw.states.OPEN)),
                pause(500),

                async(
                        sync(
                                pause(750),
                                trajectory(high_to_stack)
                        ),
                        sync(
                                pause(100),
                                inline(() -> linkage.goToLevel(80,0.3))
                        ),
                        sync(
                                inline(() -> DESIRED_ANGLE = -180),
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
//                                inline(() -> {turret.disengageSuperBrake(); turret.disengageBrake();}),
                                pause(100),
                                inline(() -> DESIRED_ANGLE = 0),
                                pause(1000)
//                                inline(() -> {turret.engageSuperBrake(); turret.engageBrake();})
                        )
                ),
                inline(() -> linkage.goToLevel(630, 0.3)),
                pause(2000),
                inline(() -> linkage.goToLevel(500, 0.3)),
                pause(1000),
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
