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

@Autonomous(name = "Left Cycles 1 + 1")
@Config
public class AutoLeftCyclesBait extends AutoBase {
    public Trajectory spline_to_high, start_to_align, high_to_stack, stack_to_high, align_to_park, strafe_left, strafe_right;

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
                .splineTo(new Vector2d(-30, -10), Math.toRadians(45))
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

        align_to_park = drive.trajectoryBuilder(stack_to_high.end(), true)
                .lineToLinearHeading(new Pose2d(-35, -35, Math.toRadians(90)))
                .build();

        strafe_left = drive.trajectoryBuilder(align_to_park.end(), true)
                .strafeLeft(25)
                .build();

        strafe_right = drive.trajectoryBuilder(align_to_park.end(), true)
                .strafeRight(25)
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
                                    linkage.goToLevel(650, 0.3);
                                })
                        )
                ),

                // Drop the linkage a little to lock the junction
                pause(100),
                async(
                        inline(() -> linkage.goToLevel(450, 0.3)),
                        sync(
                                pause(500),
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
                                inline(() -> turret.goToAngleAuto(-175, 0.5)),
                                pause(1000),
                                inline(() -> {turret.engageSuperBrake(); turret.engageBrake();})
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
                                inline(() -> turret.goToAngleAuto(20, .3)), //10
                                pause(1000),
                                inline(() -> {turret.engageSuperBrake(); turret.engageBrake();})
                        )
                ),
                inline(() -> linkage.goToLevel(670, 0.3)),
                pause(1000),
                inline(() -> linkage.goToLevel(500, 0.3)),
                pause(1000),
                inline(() -> claw.setState(Claw.states.OPEN)),
                pause(1000),
                async(
                        trajectory(align_to_park),
                        inline(() -> {
                            claw.setState(Claw.states.OPEN);
                            linkage.goToLevel(0, 1.0);
                            turret.goToAngle(0, 1.0);
                        })
                ),
                inline(() -> {
                  if(x==1){
                      drive.followTrajectory(strafe_left);
                  }
                  if(x==3){
                      drive.followTrajectory(strafe_right);
                  }
                })

        );
    }
}
