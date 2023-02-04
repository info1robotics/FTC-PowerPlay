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

@Autonomous(name = "Right Cycles 1 + 1")
@Config
public class AutoLeftCyclesBaitRight extends AutoBase {
    public Trajectory spline_to_high,
            start_to_align,
            high_to_align_stack,
            align_stack_to_stack,
            stack_to_align_stack,
            align_stack_to_high,
            align_to_park,
            strafe_left,
            strafe_right,
            high_forward,
            high_back;

    public static double result = ((9.0 * Math.sin(2 * Math.PI / 3) + Math.pow(Math.E, Math.log(Math.abs(27.73 - 9.0 * Math.cos(Math.PI / 4))) / Math.log(10)) + 2 * Math.pow(Math.E, -27.73 * Math.sin(Math.PI / 6))) / (1 + Math.pow(Math.E, 2 * Math.PI / 3)));

    public static double SPLINE_TO_HIGH_HEADING = 127.73;

    public static double SPLINE_TO_HIGH_X = 33;
    public static double SPLINE_TO_HIGH_Y = -11;

    public static double HIGH_FORWARD_X = 35.5;
    public static double HIGH_FORWARD_Y = -8;

    public static double STACK_HEADING = -180.0;

    public static double ALIGN_STACK_X = 47;
    public static double ALIGN_STACK_Y = -9.0;

    public static double STACK_X = 57.90;
    public static double STACK_Y = -9.0;

    Pose2d startPoseRight = new Pose2d(35, -62, Math.toRadians(90));
    // Velocity Constraints (just in case)
//    public TrajectorySequence stack_to_high;
//    TrajectoryVelocityConstraint slowConstraint = new MinVelocityConstraint(Arrays.asList(
//            new TranslationalVelocityConstraint(20)
//    ));
//    TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(10);

    @Override
    public void onInit() {
        drive.setPoseEstimate(startPoseRight);

        start_to_align = drive.trajectoryBuilder(startPoseRight)
                .lineTo(new Vector2d(35, -23.5))
                .build();

        spline_to_high = drive.trajectoryBuilder(start_to_align.end())
                .splineTo(new Vector2d(SPLINE_TO_HIGH_X, SPLINE_TO_HIGH_Y), Math.toRadians(SPLINE_TO_HIGH_HEADING))
                .build();

        high_to_align_stack = drive.trajectoryBuilder(spline_to_high.end(), true)
                .splineTo(new Vector2d(ALIGN_STACK_X, ALIGN_STACK_Y), Math.toRadians(STACK_HEADING))
                .build();

        align_stack_to_stack = drive.trajectoryBuilder(high_to_align_stack.end())
                .lineToConstantHeading(new Vector2d(STACK_X, STACK_Y))
                .build();

        stack_to_align_stack = drive.trajectoryBuilder(align_stack_to_stack.end())
                .lineToConstantHeading(new Vector2d(ALIGN_STACK_X, ALIGN_STACK_Y))
                .build();

        align_stack_to_high = drive.trajectoryBuilder(stack_to_align_stack.end())
                .splineTo(new Vector2d(SPLINE_TO_HIGH_X, SPLINE_TO_HIGH_Y), Math.toRadians(SPLINE_TO_HIGH_HEADING))
                .build();

        align_to_park = drive.trajectoryBuilder(
//                stack_to_high.end(),
                spline_to_high.end(), true)
                .lineTo(new Vector2d(35, -37))
                .build();

        strafe_left = drive.trajectoryBuilder(align_to_park.end(), true)
                .lineTo(new Vector2d(50, -37))
                .build();

        strafe_right = drive.trajectoryBuilder(align_to_park.end(), true)
                .lineTo(new Vector2d(10, -37))
                .build();

        high_forward = drive.trajectoryBuilder(stack_to_align_stack.end())
                .splineTo(new Vector2d(HIGH_FORWARD_X, HIGH_FORWARD_Y), Math.toRadians(SPLINE_TO_HIGH_HEADING))
                .build();

        high_back = drive.trajectoryBuilder(high_forward.end())
                .lineTo(new Vector2d(SPLINE_TO_HIGH_X, SPLINE_TO_HIGH_Y))
                .build();

        task = sync(

                // Robot goes forward and aligns itself with the mid junction.
                inline(() -> claw.setState(Claw.states.CLOSED)),
                pause(150),
                async(
                        trajectory(start_to_align),
                        sync(
                                pause(100),
                                inline(() -> {
                                    linkage.goToLevel(250, 0.3);
                                })
                        )
                ),
//
//                // The robot does a spline to align itself with the high junction.
                async(
                        trajectory(spline_to_high),
                        sync(
                                pause(100),
                                inline(() -> {
                                    turret.goToAngle(0, .3);
                                    linkage.goToLevel(650, 0.35);
                                })
                        )
                ),
////
////                // Drop the linkage a little to lock the junction
                pause(100),
                async(
                        inline(() -> linkage.goToLevel(450, 0.3)),
                        sync(
                                pause(500),
                                inline(() -> claw.setState(Claw.states.OPEN))
                        )
                ),
                pause(500),
                trajectory(align_to_park),
                async(
                        inline(() -> turret.goToAngle(0, 0.3)),
                        inline(() -> linkage.goToLevel(0, 0.3))
                ),
                inline(() -> {
                  if(x==1){
                      drive.followTrajectory(strafe_left);
                  }
                  if(x==3){
                      drive.followTrajectory(strafe_right);
                  }
                })
//                // First cycle begins here
//                trajectory(high_to_align_stack),
//                inline(() -> {
//                    turret.goToAngle(0, .3);
//                    linkage.goToLevel(125, 0.3);
//                }),
//                pause(100),
//                trajectory(align_stack_to_stack),
//                pause(500),
//                inline(() -> claw.setState(Claw.states.CLOSED)),
//                pause(200),
//                inline(() -> {
//                    linkage.goToLevel(300, 0.3);
//                }),
//                pause(100),
//                trajectory(stack_to_align_stack),
//                pause(100),
//                async(
//                        trajectory(high_forward),
//                        sync(
//                                pause(300),
//                                inline(() -> turret.goToAngle(0, .5))
//                        )
//                ),
//                inline(() -> {
//                    turret.goToAngle(0, .4);
//                }),
//                pause(2000),
//                inline(() -> {
//                    linkage.goToLevel(650, 0.55);
//                }),
//                pause(2000),
//                async(
//                        inline(() -> linkage.goToLevel(450, 0.3)),
//                        sync(
//                                pause(500),
//                                inline(() -> claw.setState(Claw.states.OPEN))
//                        )
//                ),
//                async(
//                        trajectory(high_to_stack),
//                        sync(
//                                pause(500),
//                                trajectory(high_to_stack)
//                        )
//                        sync(
//                                pause(100),
//                                inline(() -> linkage.goToLevel(120,0.3))
//                        ),
//                        sync(
//                                inline(() -> {turret.disengageSuperBrake(); turret.disengageBrake();}),
//                                pause(300),
//                                inline(() -> turret.goToAngleAuto(-175, 0.5)),
//                                pause(1000),
//                                inline(() -> {turret.engageSuperBrake(); turret.engageBrake();})
//                        )
//                )
//                pause(500),
//
//                // Secure the cone
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
//                                inline(() -> turret.goToAngleAuto(20, .3)), //10
//                                pause(1000),
//                                inline(() -> {turret.engageSuperBrake(); turret.engageBrake();})
//                        )
//                ),
//                inline(() -> linkage.goToLevel(670, 0.3)),
//                pause(1000),
//                inline(() -> linkage.goToLevel(500, 0.3)),
//                pause(1000),
//                inline(() -> claw.setState(Claw.states.OPEN)),
//                pause(1000),
//                async(
//                        trajectory(align_to_park),
//                        inline(() -> {
//                            claw.setState(Claw.states.OPEN);
//                            linkage.goToLevel(0, 1.0);
//                            turret.goToAngle(0, 1.0);
//                        })
//                ),
//                trajectory(align_to_park),
//                inline(() -> {
//                  if(x==1){
//                      drive.followTrajectory(strafe_left);
//                  }
//                  if(x==3){
//                      drive.followTrajectory(strafe_right);
//                  }
//                })
        );
    }
}
