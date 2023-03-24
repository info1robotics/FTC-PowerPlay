package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.execute;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.parallel;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.serial;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.sleepms;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.trajectorySequence;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.SubSystems.V2.Claw;

import java.util.Arrays;

@Disabled
@Autonomous(name = "Auto MID Left")
@Config
public class AutoMidLeft extends AutoBase {
    public Pose2d startPoseLeft;
    public TrajectorySequence preload_high, preload_to_stack;
    public TrajectorySequence cycle1_mid, cycle2_mid, cycle3_mid, cycle4_mid, cycle5_mid;
    public TrajectorySequence cycle1_stack, cycle2_stack, cycle3_stack, cycle4_stack;
    public TrajectorySequence zone1, zone2, zone3;
    TrajectoryVelocityConstraint slowConstraint = new MinVelocityConstraint(Arrays.asList(
            new TranslationalVelocityConstraint(50)
    ));


    TrajectoryVelocityConstraint fastConstraint = new MinVelocityConstraint(Arrays.asList(
            new TranslationalVelocityConstraint(80)
    ));

    TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(40);
    TrajectoryAccelerationConstraint fastAccelConstraint = new ProfileAccelerationConstraint(80);

    @Override
    public void onInit() {
        startPoseLeft = new Pose2d(-35, -62, Math.toRadians(90));
        drive.setPoseEstimate(startPoseLeft);

        preload_high = drive.trajectorySequenceBuilder(startPoseLeft)
                .lineToLinearHeading(new Pose2d(-36, -5.5, Math.toRadians(45)))
                .build();

        preload_to_stack = drive.trajectorySequenceBuilder(preload_high.end())
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(-57.3, -11))
                .resetConstraints()
                .build();

        cycle1_mid = drive.trajectorySequenceBuilder(preload_to_stack.end())
                .lineToLinearHeading(new Pose2d(-23, -10.5)) //21.5
                .build();

        cycle1_stack = drive.trajectorySequenceBuilder(cycle1_mid.end())
                .lineToLinearHeading(new Pose2d(-57.8, -11))
                .build();

        cycle2_mid = drive.trajectorySequenceBuilder(cycle1_stack.end())
                .lineToLinearHeading(new Pose2d(-23, -11))
                .build();

        cycle2_stack = drive.trajectorySequenceBuilder(cycle2_mid.end())
                .lineToLinearHeading(new Pose2d(-57.8, -11))
                .build();

        cycle3_mid = drive.trajectorySequenceBuilder(cycle2_stack.end())
                .lineToLinearHeading(new Pose2d(-23, -11.5))
                .build();

        cycle3_stack = drive.trajectorySequenceBuilder(cycle3_mid.end())
                .lineToLinearHeading(new Pose2d(-57.8, -11))
                .build();

        cycle4_mid = drive.trajectorySequenceBuilder(cycle3_stack.end())
                .lineToLinearHeading(new Pose2d(-23, -11.5))
                .build();

        cycle4_stack = drive.trajectorySequenceBuilder(cycle4_mid.end())
                .lineToLinearHeading(new Pose2d(-56.2, -11))
                .build();


        cycle5_mid = drive.trajectorySequenceBuilder(cycle4_stack.end())
                .lineToLinearHeading(new Pose2d(-23, -13))
                .build();

        zone1 = drive.trajectorySequenceBuilder(cycle5_mid.end())
                .setAccelConstraint(fastAccelConstraint)
                .setVelConstraint(fastConstraint)
                .lineToLinearHeading(new Pose2d(-66.5, -13, Math.toRadians(-85)))
                .resetConstraints()
                .build();

        zone2 = drive.trajectorySequenceBuilder(cycle5_mid.end())
                .setAccelConstraint(fastAccelConstraint)
                .setVelConstraint(fastConstraint)
                .lineToLinearHeading(new Pose2d(-38, -14, Math.toRadians(-85)))
                .resetConstraints()
                .build();

        zone3 = drive.trajectorySequenceBuilder(cycle5_mid.end())
                .setAccelConstraint(fastAccelConstraint)
                .setVelConstraint(fastConstraint)
                .lineToLinearHeading(new Pose2d(-9.5, -15, Math.toRadians(-85)))
                .resetConstraints()
                .build();


        task = serial(
                parallel(
                        trajectorySequence(preload_high),
                        execute((() -> targetHeight = 400)),
                        execute((() -> claw.setSubsystemState(Claw.subsystemStates.COLLECTED)))
                ),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INTERMEDIARY))),
                sleepms(300),
                execute((() -> targetHeight = 300)),
                sleepms(300),
                execute((() -> claw.setClawState(Claw.clawStates.OPEN))),
                sleepms(50),
                parallel(
                        trajectorySequence(preload_to_stack),
                                execute(() -> targetHeight = 95),
                                execute(() -> turret.setTargetAngle(-180)),
                                execute(() -> claw.setSubsystemState(Claw.subsystemStates.READY))

                ),
                execute((() -> claw.setClawState(Claw.clawStates.CLOSED))),
                sleepms(200),
//                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INTERMEDIARY))),
//                execute((() -> claw.setSubsystemState(Claw.subsystemStates.COLLECTED))),
//                sleepms(300),
                execute((() -> targetHeight = 150)),
                sleepms(200),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INTERMEDIARY))),

                // cycle 1

                parallel(
                        execute((() -> targetHeight = 325)),
                        trajectorySequence(cycle1_mid),
                        serial(
                                sleepms(200),
                        execute(() -> turret.setTargetAngle(-270))
                        )
                ),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INTERMEDIARY))),
                sleepms(300),
                execute((() -> targetHeight = 175)),
                sleepms(300),
                execute((() -> claw.setClawState(Claw.clawStates.OPEN))),
                sleepms(200),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INIT))),
//                sleepms(275),
                // high to stack

                parallel(
                        trajectorySequence(cycle1_stack),
                        serial(
                                sleepms(100),
                                execute((() -> targetHeight = 75)),
                                execute((() -> turret.setTargetAngle(-180))),
                                execute((() -> claw.setSubsystemState(Claw.subsystemStates.READY)))
                        )
                ),
                execute((() -> claw.setClawState(Claw.clawStates.CLOSED))),
                sleepms(50),
//                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INTERMEDIARY))),
                execute((() -> targetHeight = 150)),
                sleepms(300),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INTERMEDIARY))),

                // cycle 2

                parallel(
                        trajectorySequence(cycle2_mid),
                        execute((() -> targetHeight = 325)),
                        execute((() -> turret.setTargetAngle(-270)))
                ),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INTERMEDIARY))),
                sleepms(300),
                execute((() -> targetHeight = 175)),
                sleepms(300),
                execute((() -> claw.setClawState(Claw.clawStates.OPEN))),
                sleepms(200),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INIT))),
//                sleepms(275),
                // high to stack

                parallel(
                        trajectorySequence(cycle2_stack),
                        serial(
                                sleepms(100),
                                execute((() -> targetHeight = 60)),
                                execute((() -> turret.setTargetAngle(-180))),
                                execute((() -> claw.setSubsystemState(Claw.subsystemStates.READY)))
                        )
                ),
                execute((() -> claw.setClawState(Claw.clawStates.CLOSED))),
                sleepms(50),
//                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INTERMEDIARY))),
//                execute((() -> claw.setSubsystemState(Claw.subsystemStates.COLLECTED))),
//                sleepms(300),
                execute((() -> targetHeight = 150)),
                sleepms(300),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INTERMEDIARY))),

                // cycle 3

                parallel(
                        trajectorySequence(cycle3_mid),
                        execute((() -> targetHeight = 325)),
                        execute((() -> turret.setTargetAngle(-270))
                )),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INTERMEDIARY))),
                sleepms(300),
                execute((() -> targetHeight = 175)),
                sleepms(300),
                execute((() -> claw.setClawState(Claw.clawStates.OPEN))),
                sleepms(200),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INIT))),
//                sleepms(275),
                // high to stack

                parallel(
                        trajectorySequence(cycle3_stack),
                        serial(
                                sleepms(100),
                                execute((() -> targetHeight = 15)),
                                execute((() -> turret.setTargetAngle(-180))),
                                execute((() -> claw.setSubsystemState(Claw.subsystemStates.READY)))
                        )
                ),
                execute((() -> claw.setClawState(Claw.clawStates.CLOSED))),
                sleepms(50),
//                execute((() -> claw.setSubsystemState(Claw.subsystemStates.COLLECTED))),
//                sleepms(300),
                execute((() -> targetHeight = 150)),
                sleepms(300),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INTERMEDIARY))),

                // cycle 4

                parallel(
                        trajectorySequence(cycle4_mid),
                        execute((() -> targetHeight = 325)),
                        execute((() -> turret.setTargetAngle(-270)))
                ),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INTERMEDIARY))),
                sleepms(300),
                execute((() -> targetHeight = 175)),
                sleepms(300),
                execute((() -> claw.setClawState(Claw.clawStates.OPEN))),
                sleepms(200),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INIT))),
//                sleepms(275),
                // high to stack

                parallel(
                        trajectorySequence(cycle4_stack),
                        serial(
                                sleepms(100),
                                execute((() -> targetHeight = -5)),
                                execute((() -> turret.setTargetAngle(-180))),
                                execute((() -> claw.setSubsystemState(Claw.subsystemStates.READY)))
                        )
                ),
                execute((() -> claw.setClawState(Claw.clawStates.CLOSED))),
                sleepms(50),
//                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INTERMEDIARY))),
//                execute((() -> claw.setSubsystemState(Claw.subsystemStates.COLLECTED))),
//                sleepms(300),
                execute((() -> targetHeight = 150)),
                sleepms(300),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INTERMEDIARY))),

                // cycle 5

                parallel(
                        trajectorySequence(cycle5_mid),
                        serial(
                                sleepms(200),
                        execute((() -> targetHeight = 325)),
                        execute((() -> turret.setTargetAngle(-270)))
                )),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INTERMEDIARY))),
                sleepms(300),
                execute((() -> targetHeight = 175)),
                execute((() -> claw.setClawState(Claw.clawStates.OPEN))),
                sleepms(200),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INIT))),
//                sleepms(275),
                execute(() -> {
                    if (preferredZone == 1) {
                        claw.setPivotPosition(Claw.pivotPositions.INIT);
                        turret.setTargetAngle(0.0);
                        targetHeight = 0;
                        drive.followTrajectorySequence(zone1);
                    } else if (preferredZone == 2){
                        claw.setPivotPosition(Claw.pivotPositions.INIT);
                        turret.setTargetAngle(0.0);
                        targetHeight = 0;
                        drive.followTrajectorySequence(zone2);
                    } else {
                        claw.setPivotPosition(Claw.pivotPositions.INIT);
                        turret.setTargetAngle(0.0);
                        targetHeight = 0;
                        drive.followTrajectorySequence(zone3);
                    }
                })
        );
    }
}
