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

import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.SubSystems.V2.Claw;

import java.util.Arrays;

//@Disabled
@Autonomous(name = "Auto HIGH Right")
@Config
public class AutoHighRight extends AutoBase {
    public Pose2d startPoseLeft;
    public TrajectorySequence preload_high, preload_to_stack;
    public TrajectorySequence cycle1_high, cycle2_high, cycle3_high, cycle4_high, cycle5_high;
    public TrajectorySequence cycle1_stack, cycle2_stack, cycle3_stack, cycle4_stack;
    public TrajectorySequence zone1, zone2, zone3;
    TrajectoryVelocityConstraint slowConstraint = new MinVelocityConstraint(Arrays.asList(
            new TranslationalVelocityConstraint(39)
    ));


    TrajectoryVelocityConstraint fastConstraint = new MinVelocityConstraint(Arrays.asList(
            new TranslationalVelocityConstraint(70)
    ));

    TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(35);
    TrajectoryAccelerationConstraint fastAccelConstraint = new ProfileAccelerationConstraint(60);

    @Override
    public void onInit() {
        startPoseLeft = new Pose2d(35, -62, Math.toRadians(90));
        drive.setPoseEstimate(startPoseLeft);

        preload_high = drive.trajectorySequenceBuilder(startPoseLeft)
                .setAccelConstraint(fastAccelConstraint)
                .setVelConstraint(fastConstraint)
                .lineToLinearHeading(new Pose2d(34.25, -7.25, Math.toRadians(135)))
                .resetConstraints()
                .build();

        preload_to_stack = drive.trajectorySequenceBuilder(preload_high.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(56.6, -9, Math.toRadians(180)))
                .resetConstraints()
                .build();

        cycle1_high = drive.trajectorySequenceBuilder(preload_to_stack.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(31.75, -10.5, Math.toRadians(180)))
                .resetConstraints()
                .build();

        cycle1_stack = drive.trajectorySequenceBuilder(cycle1_high.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(57.1, -9, Math.toRadians(180)))
                .build();

        cycle2_high = drive.trajectorySequenceBuilder(cycle1_stack.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(31.75, -10.5, Math.toRadians(180)))
                .resetConstraints()
                .build();

        cycle2_stack = drive.trajectorySequenceBuilder(cycle2_high.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(57.1, -8.75, Math.toRadians(180)))
                .build();

        cycle3_high = drive.trajectorySequenceBuilder(cycle2_stack.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(32, -10.5, Math.toRadians(180)))
                .resetConstraints()
                .build();

        cycle3_stack = drive.trajectorySequenceBuilder(cycle3_high.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(57.5, -8.75, Math.toRadians(180)))
                .build();

        cycle4_high = drive.trajectorySequenceBuilder(cycle3_stack.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(31.5, -10.5, Math.toRadians(180)))
                .resetConstraints()
                .build();

        cycle4_stack = drive.trajectorySequenceBuilder(cycle4_high.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(57.5, -8.75, Math.toRadians(180)))
                .build();


        cycle5_high = drive.trajectorySequenceBuilder(cycle4_stack.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(32, -10.4, Math.toRadians(180)))
                .resetConstraints()
                .build();

        zone1 = drive.trajectorySequenceBuilder(cycle5_high.end())
                .setAccelConstraint(fastAccelConstraint)
                .setVelConstraint(fastConstraint)
                .lineToLinearHeading(new Pose2d(10, -13, Math.toRadians(-90)))
                .resetConstraints()
                .build();

        zone2 = drive.trajectorySequenceBuilder(cycle5_high.end())
                .setAccelConstraint(fastAccelConstraint)
                .setVelConstraint(fastConstraint)
                .lineToLinearHeading(new Pose2d(37, -12, Math.toRadians(-90)))
                .resetConstraints()
                .build();

        zone3 = drive.trajectorySequenceBuilder(cycle5_high.end())
                .setAccelConstraint(fastAccelConstraint)
                .setVelConstraint(fastConstraint)
                .lineToLinearHeading(new Pose2d(65, -15, Math.toRadians(-90)))
                .resetConstraints()
                .build();

        task = serial(
                parallel(
                        trajectorySequence(preload_high),
                        execute((() -> targetHeight = 400)),
                        execute((() -> claw.setSubsystemState(Claw.subsystemStates.COLLECTED)))
                ),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.AUTODROP))),
                sleepms(200),
                execute((() -> targetHeight = 300)),
                sleepms(100),
                execute((() -> claw.setClawState(Claw.clawStates.OPEN))),
                sleepms(100),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INIT))),
                sleepms(200),
//                sleepms(300),
                parallel(
                                trajectorySequence(preload_to_stack),
                                execute(() -> turret.setTargetAngle(180)),
                                execute(() -> claw.setSubsystemState(Claw.subsystemStates.READY)),
                                execute(() -> targetHeight = 90)
                ),
                execute((() -> claw.setClawState(Claw.clawStates.CLOSED))),
                sleepms(200),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INTERMEDIARY))),
//                execute((() -> claw.setSubsystemState(Claw.subsystemStates.COLLECTED))),
//                sleepms(300),
                execute((() -> targetHeight = 150)),
                sleepms(200),
                execute((() -> claw.setSubsystemState(Claw.subsystemStates.COLLECTED))),
                // cycle 1

                parallel(
                        execute((() -> targetHeight = 400)),
                        trajectorySequence(cycle1_high),
                        execute(() -> turret.setTargetAngle(55))
                ),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.AUTODROP))),
                sleepms(200),
                execute((() -> targetHeight = 300)),
                sleepms(100),
                execute((() -> claw.setClawState(Claw.clawStates.OPEN))),
                sleepms(100),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INIT))),
                sleepms(200),
//                sleepms(250),
                // high to stack

                parallel(
                        serial(
                                sleepms(50),
                                trajectorySequence(cycle1_stack)
                        ),
                        serial(
                                execute((() -> targetHeight = 70)),
                                execute((() -> turret.setTargetAngle(180))),
                                execute((() -> claw.setSubsystemState(Claw.subsystemStates.READY)))
                        )
                ),
                execute((() -> claw.setClawState(Claw.clawStates.CLOSED))),
                sleepms(200),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INTERMEDIARY))),
//                execute((() -> claw.setSubsystemState(Claw.subsystemStates.COLLECTED))),
//                sleepms(300),
                execute((() -> targetHeight = 150)),
                sleepms(200),
                execute((() -> claw.setSubsystemState(Claw.subsystemStates.COLLECTED))),
                // cycle 2

                parallel(
                        trajectorySequence(cycle2_high),
                        execute((() -> targetHeight = 400)),
                        execute((() -> turret.setTargetAngle(55)))
                ),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.AUTODROP))),
                sleepms(200),
                execute((() -> targetHeight = 300)),
                sleepms(100),
                execute((() -> claw.setClawState(Claw.clawStates.OPEN))),
                sleepms(100),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INIT))),
                sleepms(200),
//                sleepms(250),
                // high to stack

                parallel(
                        trajectorySequence(cycle2_stack),
                        execute((() -> targetHeight = 55)),
                        execute((() -> turret.setTargetAngle(180))),
                        execute((() -> claw.setSubsystemState(Claw.subsystemStates.READY)))
                ),
                execute((() -> claw.setClawState(Claw.clawStates.CLOSED))),
                sleepms(250),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INTERMEDIARY))),
//                execute((() -> claw.setSubsystemState(Claw.subsystemStates.COLLECTED))),
//                sleepms(300),
                execute((() -> targetHeight = 150)),
                sleepms(200),
                execute((() -> claw.setSubsystemState(Claw.subsystemStates.COLLECTED))),

                // cycle 3

                parallel(
                        trajectorySequence(cycle3_high),
                        execute((() -> targetHeight = 400)),
                        execute((() -> turret.setTargetAngle(55)))
                ),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.AUTODROP))),
                sleepms(200),
                execute((() -> targetHeight = 300)),
                sleepms(100),
                execute((() -> claw.setClawState(Claw.clawStates.OPEN))),
                sleepms(100),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INIT))),
                sleepms(200),
//                sleepms(250),
                // high to stack

                parallel(
                        trajectorySequence(cycle3_stack),
                        execute((() -> targetHeight = 0)),
                        execute((() -> turret.setTargetAngle(180))),
                        execute((() -> claw.setSubsystemState(Claw.subsystemStates.READY)))
                ),
                execute((() -> claw.setClawState(Claw.clawStates.CLOSED))),
                sleepms(250),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INTERMEDIARY))),
//                execute((() -> claw.setSubsystemState(Claw.subsystemStates.COLLECTED))),
//                sleepms(300),
                execute((() -> targetHeight = 150)),
                sleepms(200),
                execute((() -> claw.setSubsystemState(Claw.subsystemStates.COLLECTED))),

                // cycle 4

                parallel(
                        trajectorySequence(cycle4_high),
                        execute((() -> targetHeight = 400)),
                        execute((() -> turret.setTargetAngle(55)))
                ),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.AUTODROP))),
                sleepms(200),
                execute((() -> targetHeight = 300)),
                sleepms(100),
                execute((() -> claw.setClawState(Claw.clawStates.OPEN))),
                sleepms(100),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INIT))),
                sleepms(200),
//                sleepms(250),
                // high to stack

                parallel(
                        trajectorySequence(cycle4_stack),
                                execute((() -> targetHeight = -10)),
                                execute((() -> turret.setTargetAngle(180))),
                                execute((() -> claw.setSubsystemState(Claw.subsystemStates.READY)))
                ),
                execute((() -> claw.setClawState(Claw.clawStates.CLOSED))),
                sleepms(250),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INTERMEDIARY))),
//                execute((() -> claw.setSubsystemState(Claw.subsystemStates.COLLECTED))),
//                sleepms(300),
                execute((() -> targetHeight = 150)),
                sleepms(200),
                execute((() -> claw.setSubsystemState(Claw.subsystemStates.COLLECTED))),

                // cycle 5

                parallel(
                        trajectorySequence(cycle5_high),
                        execute((() -> targetHeight = 400)),
                        execute((() -> turret.setTargetAngle(55)))
                ),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.AUTODROP))),
                sleepms(200),
                execute((() -> targetHeight = 300)),
                sleepms(100),
                execute((() -> claw.setClawState(Claw.clawStates.OPEN))),
                sleepms(100),
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
