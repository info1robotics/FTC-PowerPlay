package org.firstinspires.ftc.teamcode.OpModes.Autonomous;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.execute;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.parallel;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.serial;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.sleepms;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.trajectory;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.trajectorySequence;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.SubSystems.V2.Claw;

@Autonomous(name = "AutoOwnership70")
@Config
public class AutoOwnership70 extends AutoBase {
    public Pose2d startPoseLeft;
    public TrajectorySequence align_preload_high_left;
    public Trajectory stack1, stack2, stack3, stack4, stack5;
    public Trajectory cycle2_mid_left, cycle3_high_middle, cycle4_mid_right, cycle5_high_right;
    @Override
    public void onInit() {
        startPoseLeft = new Pose2d(-35, -62, Math.toRadians(90));
        drive.setPoseEstimate(startPoseLeft);

        align_preload_high_left = drive.trajectorySequenceBuilder(startPoseLeft)
                .lineToLinearHeading(new Pose2d(-33.5, -7.5, Math.toRadians(0)))
                .build();

        stack1 = drive.trajectoryBuilder(align_preload_high_left.end())
                .lineToLinearHeading(new Pose2d(-59, -9, Math.toRadians(0)))
                .build();

        cycle2_mid_left = drive.trajectoryBuilder(stack1.end())
                .lineToLinearHeading(new Pose2d(-30.5, -10, Math.toRadians(0)))
                .build();

        stack2 = drive.trajectoryBuilder(cycle2_mid_left.end())
                .lineToLinearHeading(new Pose2d(-60, -9, Math.toRadians(0)))
                .build();

        cycle3_high_middle = drive.trajectoryBuilder(stack2.end())
                .lineToLinearHeading(new Pose2d(-6, -10, Math.toRadians(0)))
                .build();

        stack3 = drive.trajectoryBuilder(cycle3_high_middle.end())
                .lineToLinearHeading(new Pose2d(-61.5, -9, Math.toRadians(0)))
                .build();

        cycle4_mid_right = drive.trajectoryBuilder(stack3.end())
                .lineToLinearHeading(new Pose2d(9, -12, Math.toRadians(0)))
                .build();

        stack4 = drive.trajectoryBuilder(cycle4_mid_right.end())
                .lineToLinearHeading(new Pose2d(-67.5, -10, Math.toRadians(0)))
                .build();

        cycle5_high_right = drive.trajectoryBuilder(stack3.end())
                .lineToLinearHeading(new Pose2d(7, -11.5, Math.toRadians(0)))
                .build();

        stack5 = drive.trajectoryBuilder(cycle5_high_right.end())
                .lineToLinearHeading(new Pose2d(47.25, -12, Math.toRadians(0)))
                .build();

        task = serial(
                parallel(
                        trajectorySequence(align_preload_high_left),
                        serial(
                                execute(() -> { turret.setTargetAngle(45); }),
                                sleepms(500),
                                execute(() -> { targetHeight = 470; }),
                                execute(() -> { claw.setSubsystemState(Claw.subsystemStates.EXTENDED_DROP); })
                        )
                ),
                sleepms(100),
                execute(() -> { claw.setClawState(Claw.clawStates.OPEN); }),
//                serial(
//                        execute(() -> { claw.setPivotPosition(Claw.pivotPositions.ANGLE_DROP); }),
//                        sleepms(300),
//                        execute(() -> { targetHeight = 200; }),
//                        sleepms(300),
//                        execute(() -> { claw.pivotMain.setPosition(Claw.PIVOT_MAIN_VERTICAL); }),
//                ),
                sleepms(100),
                execute(() -> { claw.setSubsystemState(Claw.subsystemStates.READY); }),
                sleepms(100),
                parallel(
                        trajectory(stack1),
                        execute(() -> { turret.setTargetAngle(180); }),
                        execute(() -> { targetHeight = 55; })
                ),
//                sleepms(200),
                execute(() -> { claw.setClawState(Claw.clawStates.CLOSED); }),
                sleepms(150),
                execute(() -> { targetHeight = 130; }),
                sleepms(200),
                parallel(
                        execute(() -> { turret.setTargetAngle(297.5); }),
                        execute(() -> { claw.setSubsystemState(Claw.subsystemStates.EXTENDED_DROP); })
                ),
                sleepms(1000),
                execute(() -> { claw.setClawState(Claw.clawStates.OPEN); }),
                sleepms(1900),
                parallel(
                        execute(() -> { turret.setTargetAngle(180); }),
                        execute(() -> { claw.setSubsystemState(Claw.subsystemStates.READY); })
                ),
                sleepms(400),
                execute(() -> { targetHeight = 35; }),
                sleepms(400),
                execute(() -> { claw.setClawState(Claw.clawStates.CLOSED); }),
                sleepms(200),
                execute(() -> { targetHeight = 120; }),
                sleepms(300),
                parallel(
                        trajectory(cycle2_mid_left),
                        serial(
                                execute(() -> { claw.setSubsystemState(Claw.subsystemStates.EXTENDED_DROP); }),
                                execute(() -> { turret.setTargetAngle(295); }),
                                sleepms(500),
                                execute(() -> { targetHeight = 280; })
                                )
                ),
                sleepms(100),
                execute(() -> { claw.setClawState(Claw.clawStates.OPEN); }),
                sleepms(100),
                execute(() -> { claw.setSubsystemState(Claw.subsystemStates.READY); }),
                parallel(
                        trajectory(stack2),
                        execute(() -> { turret.setTargetAngle(180); }),
                        execute(() -> { targetHeight = 15; })
                ),
                execute(() -> { claw.setClawState(Claw.clawStates.CLOSED); }),
                sleepms(200),
                execute(() -> { targetHeight = 120; }),
                sleepms(200),
                parallel(
                        trajectory(cycle3_high_middle),
                        serial(
                                execute(() -> { claw.setSubsystemState(Claw.subsystemStates.EXTENDED_DROP); }),
                                execute(() -> { turret.setTargetAngle(297.5); }),
                                sleepms(750),
                                execute(() -> { targetHeight = 250; }),
                                sleepms(500),
                                execute(() -> { targetHeight = 475; })
                        )
                ),
                sleepms(100),
                execute(() -> { claw.setClawState(Claw.clawStates.OPEN); }),
                sleepms(100),
                execute(() -> { claw.setSubsystemState(Claw.subsystemStates.READY); }),
                parallel(
                        trajectory(stack3),
                        execute(() -> { turret.setTargetAngle(180); }),
                        execute(() -> { targetHeight = -10; })
                ),
                execute(() -> { claw.setClawState(Claw.clawStates.CLOSED); }),
                sleepms(200),
                execute(() -> { targetHeight = 120; }),
                sleepms(200),
                parallel(
                        trajectory(cycle4_mid_right),
                        serial(
                                execute(() -> { claw.setSubsystemState(Claw.subsystemStates.EXTENDED_DROP); }),
                                execute(() -> { turret.setTargetAngle(297.5); }),
                                sleepms(1000),
                                execute(() -> { targetHeight = 500; }),
                                sleepms(500),
                                execute(() -> { targetHeight = 280; })
                        )
                ),
                sleepms(100),
                execute(() -> { claw.setClawState(Claw.clawStates.OPEN); }),
                sleepms(100),
                execute(() -> { claw.setSubsystemState(Claw.subsystemStates.READY); }),
                parallel(
                        trajectory(stack4),
                        execute(() -> { turret.setTargetAngle(180); }),
                        execute(() -> { targetHeight = -50; })
                ),
                execute(() -> { claw.setClawState(Claw.clawStates.CLOSED); }),
                sleepms(200),
                execute(() -> { targetHeight = 120; }),
                sleepms(200),
                parallel(
                        trajectory(cycle5_high_right),
                        serial(
                                execute(() -> { claw.setSubsystemState(Claw.subsystemStates.EXTENDED_DROP); }),
                                execute(() -> { turret.setTargetAngle(62.5); }),
                                sleepms(1000),
                                execute(() -> { targetHeight = 475; })
                        )
                ),
                execute(() -> { claw.setClawState(Claw.clawStates.OPEN); }),
                sleepms(100),
                execute(() -> { claw.setSubsystemState(Claw.subsystemStates.READY); }),
                sleepms(100),
                parallel(
                        trajectory(stack5),
                        execute(() -> { turret.setTargetAngle(0); }),
                        execute(() -> { targetHeight = 55; })
                ),
                execute(() -> { claw.setClawState(Claw.clawStates.CLOSED); }),
                sleepms(150),
                execute(() -> { targetHeight = 130; }),
                sleepms(200),
                parallel(
                        execute(() -> { turret.setTargetAngle(-62.5); }),
                        execute(() -> { claw.setSubsystemState(Claw.subsystemStates.EXTENDED_DROP); })
                ),
                sleepms(900),
                execute(() -> { claw.setClawState(Claw.clawStates.OPEN); }),
                sleepms(300)
                );
    }
}
