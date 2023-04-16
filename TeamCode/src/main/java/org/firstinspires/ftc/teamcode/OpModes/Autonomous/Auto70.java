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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.SubSystems.V2.Claw;
@Disabled
@Autonomous(name = "Auto70")
@Config
public class Auto70 extends AutoBase {
    public Pose2d startPoseLeft;
    public TrajectorySequence align_preload;
    public Trajectory stack1, stack2, stack3, stack4, stack5;
    public Trajectory high1, high2, high3, high4, high5;
    @Override
    public void onInit() {
        startPoseLeft = new Pose2d(-35, -62, Math.toRadians(90));
        drive.setPoseEstimate(startPoseLeft);

        align_preload = drive.trajectorySequenceBuilder(startPoseLeft)
                .lineToLinearHeading(new Pose2d(-35, -9, Math.toRadians(0)))
                .build();

        stack1 = drive.trajectoryBuilder(align_preload.end())
                .lineToLinearHeading(new Pose2d(-58, -8, Math.toRadians(0)))
                .build();

        high1 = drive.trajectoryBuilder(stack1.end())
                .lineToLinearHeading(new Pose2d(-35, -9, Math.toRadians(0)))
                .build();

        stack2 = drive.trajectoryBuilder(high1.end())
                .lineToLinearHeading(new Pose2d(-58.25, -8, Math.toRadians(0)))
                .build();

        high2 = drive.trajectoryBuilder(stack2.end())
                .lineToLinearHeading(new Pose2d(-35, -9, Math.toRadians(0)))
                .build();

        stack3 = drive.trajectoryBuilder(high2.end())
                .lineToLinearHeading(new Pose2d(-58.25, -8, Math.toRadians(0)))
                .build();

        high3 = drive.trajectoryBuilder(stack3.end())
                .lineToLinearHeading(new Pose2d(-35, -10, Math.toRadians(0)))
                .build();

        stack4 = drive.trajectoryBuilder(high3.end())
                .lineToLinearHeading(new Pose2d(-58, -8, Math.toRadians(0)))
                .build();

        high4 = drive.trajectoryBuilder(stack4.end())
                .lineToLinearHeading(new Pose2d(-35, -11, Math.toRadians(0)))
                .build();

        stack5 = drive.trajectoryBuilder(high4.end())
                .lineToLinearHeading(new Pose2d(-58, -8, Math.toRadians(0)))
                .build();

        high5 = drive.trajectoryBuilder(stack5.end())
                .lineToLinearHeading(new Pose2d(-35, -11, Math.toRadians(0)))
                .build();

        task = serial(
                parallel(
                        trajectorySequence(align_preload),
                        serial(
                                execute(() -> { turret.setTargetAngle(45); }),
                                sleepms(500),
                                execute(() -> { targetHeight = 430; }),
                                execute(() -> { claw.setSubsystemState(Claw.subsystemStates.EXTENDED); })
                        )
                ),
                serial(
                        execute(() -> { claw.setPivotPosition(Claw.pivotPositions.ANGLE_DROP); }),
                        sleepms(300),
                        execute(() -> { targetHeight = 200; }),
                        sleepms(300),
                        execute(() -> { claw.pivotMain.setPosition(Claw.PIVOT_MAIN_VERTICAL); }),
                        execute(() -> { claw.setClawState(Claw.clawStates.OPEN); })
                ),
                sleepms(100),
                execute(() -> { claw.setSubsystemState(Claw.subsystemStates.READY); }),
                sleepms(100),

                // 1st cycle

                parallel(
                        trajectory(stack1),
                        execute(() -> { turret.setTargetAngle(180); }),
                        execute(() -> { targetHeight = 95; })
                ),
                sleepms(200),
                execute(() -> { claw.setClawState(Claw.clawStates.CLOSED); }),
                sleepms(200),
                execute(() -> { targetHeight = 150; }),
                sleepms(500),
                parallel(
                        serial(
                                execute(() -> { claw.setSubsystemState(Claw.subsystemStates.EXTENDED); }),
                                execute(() -> { turret.setTargetAngle(42.5); }),
                                sleepms(500),
                                execute(() -> { targetHeight = 430; })
                                ),
                        trajectory(high1)
                ),
                sleepms(100),
                serial(
                        execute(() -> { claw.setPivotPosition(Claw.pivotPositions.ANGLE_DROP); }),
                        sleepms(300),
                        execute(() -> { targetHeight = 200; }),
                        sleepms(300),
                        execute(() -> { claw.pivotMain.setPosition(Claw.PIVOT_MAIN_VERTICAL); }),
                        execute(() -> { claw.setClawState(Claw.clawStates.OPEN); })
                ),
                sleepms(100),
                execute(() -> { claw.setSubsystemState(Claw.subsystemStates.READY); }),
                sleepms(100),

                // 2nd cycle

                parallel(
                        trajectory(stack2),
                        execute(() -> { turret.setTargetAngle(180); }),
                        execute(() -> { targetHeight = 60; })
                ),
                sleepms(200),
                execute(() -> { claw.setClawState(Claw.clawStates.CLOSED); }),
                sleepms(200),
                execute(() -> { targetHeight = 150; }),
                sleepms(500),
                parallel(
                        serial(
                                execute(() -> { claw.setSubsystemState(Claw.subsystemStates.EXTENDED); }),
                                execute(() -> { turret.setTargetAngle(42.5); }),
                                sleepms(500),
                                execute(() -> { targetHeight = 430; })
                        ),
                        trajectory(high2)
                ),
                sleepms(100),
                serial(
                        execute(() -> { claw.setPivotPosition(Claw.pivotPositions.ANGLE_DROP); }),
                        sleepms(300),
                        execute(() -> { targetHeight = 200; }),
                        sleepms(300),
                        execute(() -> { claw.pivotMain.setPosition(Claw.PIVOT_MAIN_VERTICAL); }),
                        execute(() -> { claw.setClawState(Claw.clawStates.OPEN); })
                ),
                sleepms(100),
                execute(() -> { claw.setSubsystemState(Claw.subsystemStates.READY); }),
                sleepms(100),

                // 3rd cycle

                parallel(
                        trajectory(stack3),
                        execute(() -> { turret.setTargetAngle(180); }),
                        execute(() -> { targetHeight = 40; })
                ),
                sleepms(200),
                execute(() -> { claw.setClawState(Claw.clawStates.CLOSED); }),
                sleepms(200),
                execute(() -> { targetHeight = 150; }),
                sleepms(500),
                parallel(
                        serial(
                                execute(() -> { claw.setSubsystemState(Claw.subsystemStates.EXTENDED); }),
                                execute(() -> { turret.setTargetAngle(45); }),
                                sleepms(500),
                                execute(() -> { targetHeight = 430; })
                        ),
                        trajectory(high3)
                ),
                sleepms(100),
                serial(
                        execute(() -> { claw.setPivotPosition(Claw.pivotPositions.ANGLE_DROP); }),
                        sleepms(300),
                        execute(() -> { targetHeight = 200; }),
                        sleepms(300),
                        execute(() -> { claw.pivotMain.setPosition(Claw.PIVOT_MAIN_VERTICAL); }),
                        execute(() -> { claw.setClawState(Claw.clawStates.OPEN); })
                ),
                sleepms(100),
                execute(() -> { claw.setSubsystemState(Claw.subsystemStates.READY); }),
                sleepms(100),

                // 4th cycle

                parallel(
                        trajectory(stack4),
                        execute(() -> { turret.setTargetAngle(180); }),
                        execute(() -> { targetHeight = 15; })
                ),
                sleepms(200),
                execute(() -> { claw.setClawState(Claw.clawStates.CLOSED); }),
                sleepms(200),
                execute(() -> { targetHeight = 150; }),
                sleepms(500),
                parallel(
                        serial(
                                execute(() -> { claw.setSubsystemState(Claw.subsystemStates.EXTENDED); }),
                                execute(() -> { turret.setTargetAngle(47); }),
                                sleepms(500),
                                execute(() -> { targetHeight = 430; })
                        ),
                        trajectory(high4)
                ),
                sleepms(100),
                serial(
                        execute(() -> { claw.setPivotPosition(Claw.pivotPositions.ANGLE_DROP); }),
                        sleepms(300),
                        execute(() -> { targetHeight = 200; }),
                        sleepms(300),
                        execute(() -> { claw.pivotMain.setPosition(Claw.PIVOT_MAIN_VERTICAL); }),
                        execute(() -> { claw.setClawState(Claw.clawStates.OPEN); })
                ),
                sleepms(100),
                execute(() -> { claw.setSubsystemState(Claw.subsystemStates.READY); }),
                sleepms(100),

                // 5th cycle

                parallel(
                        trajectory(stack5),
                        execute(() -> { turret.setTargetAngle(180); }),
                        execute(() -> { targetHeight = 0; })
                ),
                sleepms(200),
                execute(() -> { claw.setClawState(Claw.clawStates.CLOSED); }),
                sleepms(200),
                execute(() -> { targetHeight = 150; }),
                sleepms(500),
                parallel(
                        serial(
                                execute(() -> { claw.setSubsystemState(Claw.subsystemStates.EXTENDED); }),
                                execute(() -> { turret.setTargetAngle(47); }),
                                sleepms(500),
                                execute(() -> { targetHeight = 430; })
                        ),
                        trajectory(high5)
                ),
                sleepms(100),
                serial(
                        execute(() -> { claw.setPivotPosition(Claw.pivotPositions.ANGLE_DROP); }),
                        sleepms(300),
                        execute(() -> { targetHeight = 200; }),
                        sleepms(300),
                        execute(() -> { claw.pivotMain.setPosition(Claw.PIVOT_MAIN_VERTICAL); }),
                        execute(() -> { claw.setClawState(Claw.clawStates.OPEN); })
                ),
                sleepms(100)
        );
    }
}
