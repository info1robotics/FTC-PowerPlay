package org.firstinspires.ftc.teamcode.OpModes.Autonomous;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.execute;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.parallel;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.serial;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.sleepms;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.trajectory;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.SubSystems.V2.Claw;

@Autonomous(name = "Auto75")
@Config
public class Auto75 extends AutoBase {
    public Pose2d startPoseLeft;
    public Trajectory align_preload;
    public Trajectory stack1, stack2, stack3, stack4, stack5;
    public Trajectory high1, high2, high3, high4, high5;
    @Override
    public void onInit() {
        startPoseLeft = new Pose2d(-35, -62, Math.toRadians(90));
        drive.setPoseEstimate(startPoseLeft);

        // Chass
        align_preload = drive.trajectoryBuilder(startPoseLeft)
                .lineToLinearHeading(new Pose2d(-35, -8, Math.toRadians(0)))
                .build();

        stack1 = drive.trajectoryBuilder(align_preload.end())
                .lineToLinearHeading(new Pose2d(-54.85, -10, Math.toRadians(0)))
                .build();

        high1 = drive.trajectoryBuilder(stack1.end())
                .lineToLinearHeading(new Pose2d(-35, -10, Math.toRadians(0)))
                .build();

        task = serial(
                parallel(
                        trajectory(align_preload),
                        serial(
                                execute(() -> { turret.setTargetAngle(45); }),
                                sleepms(500),
                                execute(() -> { targetHeight = 430; }),
                                execute(() -> { claw.setSubsystemState(Claw.subsystemStates.EXTENDED); })
                        )
                ),
                parallel(
                        execute(() -> { claw.setSubsystemState(Claw.subsystemStates.EXTENDED_DROP); }),
                        execute(() -> { targetHeight = 300; })
                ),
                sleepms(100),
                parallel(
                        trajectory(stack1),
                        execute(() -> { turret.setTargetAngle(180); }),
                        execute(() -> { targetHeight = 70; }),
                        execute(() -> { claw.setSubsystemState(Claw.subsystemStates.READY); })
                ),
                execute(() -> { claw.setSubsystemState(Claw.subsystemStates.EXTENDED); }),
                sleepms(100),
                parallel(
                        serial(
                                execute(() -> { targetHeight = 200; }),
                                execute(() -> { claw.setSubsystemState(Claw.subsystemStates.EXTENDED); }),
                                execute(() -> { turret.setTargetAngle(45); }),
                                sleepms(350),
                                execute(() -> { targetHeight = 430; })
                                ),
                        trajectory(high1)
                ),
                sleepms(300),
                parallel(
                        execute(() -> { claw.setSubsystemState(Claw.subsystemStates.EXTENDED_DROP); }),
                        execute(() -> { targetHeight = 300; })
                ),
                sleepms(1000)
        );
    }
}
