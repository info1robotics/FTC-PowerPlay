package org.firstinspires.ftc.teamcode.OpModes;


import static org.firstinspires.ftc.teamcode.SubSystems.Linkage.DESIRED_HEIGHT;
import static org.firstinspires.ftc.teamcode.SubSystems.Turret.DESIRED_ANGLE;
import static org.firstinspires.ftc.teamcode.SubSystems.Turret.AUTO_SPEED;
import static org.firstinspires.ftc.teamcode.SubSystems.Turret.REVERT_THRESHOLD;
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

import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;

import java.util.Arrays;

//@Disabled
@Autonomous(name = "Left Cycles Low")
@Config
public class AutoLeftCyclesLow extends AutoBase {
    public Pose2d startPoseLeft;
    public Trajectory align_preload;
    public TrajectorySequence stack_to_low, preload_to_turn, low_to_stack, turn_to_stack, go_to_stack, low_to_stack_1, low_to_stack_2, low_to_stack_3, low_to_stack_4;
    public TrajectorySequence run_to_zone_3, run_to_zone_2, run_to_zone_1;
    public static double HIGH_TURRET_ANGLE = -95;

    TrajectoryVelocityConstraint slowConstraint = new MinVelocityConstraint(Arrays.asList(
            new TranslationalVelocityConstraint(40),
            new AngularVelocityConstraint(2)
    ));

    TrajectoryVelocityConstraint fastConstraint = new MinVelocityConstraint(Arrays.asList(
            new TranslationalVelocityConstraint(80)
    ));
    TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(20);
    TrajectoryAccelerationConstraint fastAccelConstraint = new ProfileAccelerationConstraint(80);



    @Override
    public void onInit() {
        startPoseLeft = new Pose2d(-37.5, -62, Math.toRadians(90));
        drive.setPoseEstimate(startPoseLeft);
        align_preload = drive.trajectoryBuilder(startPoseLeft)
                .lineTo(new Vector2d(startPoseLeft.getX()-1, -20))
                .build();

        preload_to_turn = drive.trajectorySequenceBuilder(align_preload.end())
                .lineTo(new Vector2d(startPoseLeft.getX()-1, -8))
                .build();

        turn_to_stack = drive.trajectorySequenceBuilder(preload_to_turn.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
            .turn(Math.toRadians(90))
                .resetConstraints()
                .build();

        go_to_stack = drive.trajectorySequenceBuilder(turn_to_stack.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineTo(new Vector2d(-58.5, -9.5))
                .resetConstraints()
                .build();

        stack_to_low = drive.trajectorySequenceBuilder(go_to_stack.end())
                .setReversed(true)
                .lineTo(new Vector2d(-50, -7))
                .build();

        low_to_stack = drive.trajectorySequenceBuilder(stack_to_low.end())
                .lineTo(new Vector2d(-59.25, -9.5))
                .build();

        low_to_stack_1 = drive.trajectorySequenceBuilder(stack_to_low.end())
                .lineTo(new Vector2d(-59, -10.5))
                .build();

        low_to_stack_2 = drive.trajectorySequenceBuilder(stack_to_low.end())
                .lineTo(new Vector2d(-58.75, -10.5))
                .build();

        low_to_stack_3 = drive.trajectorySequenceBuilder(stack_to_low.end())
                .lineTo(new Vector2d(-58.75, -10.5))
                .build();

        low_to_stack_4 = drive.trajectorySequenceBuilder(stack_to_low.end())
                .lineTo(new Vector2d(-58.75, -8))
                .build();

        run_to_zone_3 = drive.trajectorySequenceBuilder(stack_to_low.end())
                .setAccelConstraint(fastAccelConstraint)
                .setVelConstraint(fastConstraint)
                .setReversed(true)
                .lineTo(new Vector2d(-14, -8))
                .resetConstraints()
                .build();

        run_to_zone_2 = drive.trajectorySequenceBuilder(stack_to_low.end())
                .setAccelConstraint(fastAccelConstraint)
                .setVelConstraint(fastConstraint)
                .setReversed(true)
                .lineTo(new Vector2d(-35, -8))
                .resetConstraints()
                .build();

        run_to_zone_1 = drive.trajectorySequenceBuilder(stack_to_low.end())
                .setAccelConstraint(fastAccelConstraint)
                .setVelConstraint(fastConstraint)
                .setReversed(true)
                .lineTo(new Vector2d(-60, -8))
                .resetConstraints()
                .build();

        task = sync(
                // Robot goes forward and aligns itself with the mid junction.
                inline(() -> claw.setState(Claw.states.CLOSED)),
                pause(200),
                inline(() -> DESIRED_HEIGHT = 400),
                async(
                trajectory(align_preload),
                inline(() -> DESIRED_ANGLE = 90)
                ),
                sync(
                        pause(250),
                        inline(() -> DESIRED_HEIGHT = 250),
                        inline(() -> claw.setState(Claw.states.OPEN))
                ),
                async(
                        trajectory(preload_to_turn),
                        sync(
                                pause(100),
                                inline(() -> DESIRED_HEIGHT = 90)
                        )
                ),

                // first cycle
                inline(() -> REVERT_THRESHOLD = 0),
                inline(() -> AUTO_SPEED = 0.1),
                inline(() -> DESIRED_ANGLE = 2.5),
                pause(300),
                trajectory(go_to_stack),
                pause(100),
                inline(() -> claw.setState(Claw.states.CLOSED)),
                inline(() -> DESIRED_HEIGHT = 200),
                pause(200),
                async(
                        trajectory(stack_to_low),
                        sync(
                                pause(200),
                                inline(() -> DESIRED_HEIGHT = 290),
                                inline(() -> AUTO_SPEED = 0.20),
                                inline(() -> {
                                    DESIRED_ANGLE = HIGH_TURRET_ANGLE;
                                    turret.setDistanceThreshold(150);
                                }),
                                pause(1500)
                                )
                ),
                inline(() -> turret.toggleSetThreshold(true)),
                pause(750),
                inline(() -> DESIRED_HEIGHT = 150),
                inline(() -> claw.setState(Claw.states.OPEN)),
                inline(() -> turret.toggleSetThreshold(false)),

                // second cycle
                inline(() -> REVERT_THRESHOLD = 0),
                inline(() -> DESIRED_ANGLE = 2.5),
                pause(750),
                inline(() -> DESIRED_HEIGHT = 70),
                pause(300),
                trajectory(low_to_stack_1),
                pause(100),
                inline(() -> claw.setState(Claw.states.CLOSED)),
                pause(100),
                inline(() -> DESIRED_HEIGHT = 200),
                async(
                        trajectory(stack_to_low),
                        sync(
                                pause(200),
                                inline(() -> DESIRED_HEIGHT = 290),
                                inline(() -> AUTO_SPEED = 0.20),
                                inline(() -> {
                                    REVERT_THRESHOLD = 0;
                                    DESIRED_ANGLE = HIGH_TURRET_ANGLE;
                                    turret.setDistanceThreshold(150);
                                }),
                                pause(1500)
                        )
                ),
                inline(() -> turret.toggleSetThreshold(true)),
                pause(750),
                inline(() -> DESIRED_HEIGHT = 150),
                inline(() -> claw.setState(Claw.states.OPEN)),
                inline(() -> turret.toggleSetThreshold(false)),
                // third cycle
                inline(() -> REVERT_THRESHOLD = 0),

                inline(() -> DESIRED_ANGLE = 2.5),
                pause(750),
                inline(() -> DESIRED_HEIGHT = 50),
                pause(300),
                trajectory(low_to_stack_2),
                pause(100),
                inline(() -> claw.setState(Claw.states.CLOSED)),
                pause(200),
                inline(() -> DESIRED_HEIGHT = 200),
                pause(200),
                async(
                        trajectory(stack_to_low),
                        sync(
                                pause(200),
                                inline(() -> DESIRED_HEIGHT = 290),
                                inline(() -> AUTO_SPEED = 0.20),
                                inline(() -> {
                                    DESIRED_ANGLE = HIGH_TURRET_ANGLE;
                                    turret.setDistanceThreshold(150);
                                }),
                                pause(1500)
                        )
                ),
                inline(() -> turret.toggleSetThreshold(true)),
                pause(750),
                inline(() -> DESIRED_HEIGHT = 150),
                inline(() -> claw.setState(Claw.states.OPEN)),
                inline(() -> turret.toggleSetThreshold(false)),

                // fourth cycle

                inline(() -> DESIRED_ANGLE = 2.5),
                pause(750),
                inline(() -> DESIRED_HEIGHT = 40),
                pause(300),
                trajectory(low_to_stack_3),
                pause(100),
                inline(() -> claw.setState(Claw.states.CLOSED)),
                pause(200),
                inline(() -> DESIRED_HEIGHT = 200),
                pause(100),
                async(
                        trajectory(stack_to_low),
                        sync(
                                pause(200),
                                inline(() -> DESIRED_HEIGHT = 290),
                                inline(() -> {
                                    DESIRED_ANGLE = HIGH_TURRET_ANGLE;
                                    turret.setDistanceThreshold(150);
                                }),
                                pause(1500)
                        )
                ),
                inline(() -> turret.toggleSetThreshold(true)),
                pause(750),
                inline(() -> DESIRED_HEIGHT = 150),
                inline(() -> claw.setState(Claw.states.OPEN)),
                inline(() -> turret.toggleSetThreshold(false)),
                async(
                        inline(() -> {
                            if(x == 2) drive.followTrajectorySequence(run_to_zone_2);
                            else if (x == 3) drive.followTrajectorySequence(run_to_zone_3);
                            else drive.followTrajectorySequence(run_to_zone_1);
                        }),
                        inline(() -> {
                            AUTO_SPEED = 0.3;
                            DESIRED_ANGLE = 0;
                        }),
                        pause(200),
                        inline(() -> DESIRED_HEIGHT = 0)
                )
        );
    }
}
