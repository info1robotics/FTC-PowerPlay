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
@Autonomous(name = "Auto MID Left")
@Config
public class AutoMidLeft extends AutoBase {
    public Pose2d startPoseLeft;
    public TrajectorySequence preload_mid, preload_to_stack;
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
        preload_mid = drive.trajectorySequenceBuilder(startPoseLeft)
                .lineToLinearHeading(new Pose2d(-35, -12, Math.toRadians(0)))
                .build();

        preload_to_stack = drive.trajectorySequenceBuilder(preload_mid.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(-56, -8.75))
                .build();

        cycle1_mid = drive.trajectorySequenceBuilder(preload_to_stack.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(-30, -12.5))
                .resetConstraints()
                .build();

        cycle1_stack = drive.trajectorySequenceBuilder(cycle1_mid.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(-56, -8.75))
                .build();

        cycle2_mid = drive.trajectorySequenceBuilder(cycle1_stack.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(-30, -12.5))
                .resetConstraints()
                .build();

        cycle2_stack = drive.trajectorySequenceBuilder(cycle2_mid.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(-56.25, -9.25))
                .build();

        cycle3_mid = drive.trajectorySequenceBuilder(cycle2_stack.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(-30, -12.5))
                .resetConstraints()
                .build();

        cycle3_stack = drive.trajectorySequenceBuilder(cycle3_mid.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(-56.25, -9.75))
                .build();

        cycle4_mid = drive.trajectorySequenceBuilder(cycle3_stack.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(-30, -13.2))
                .resetConstraints()
                .build();

        cycle4_stack = drive.trajectorySequenceBuilder(cycle4_mid.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(-56.25, -9.75))
                .build();


        cycle5_mid = drive.trajectorySequenceBuilder(cycle4_stack.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(-30, -13.2))
                .resetConstraints()
                .build();


        task = serial(
                parallel(
                        trajectorySequence(preload_mid),
                        execute((() -> targetHeight = 400)),
                        execute((() -> claw.setSubsystemState(Claw.subsystemStates.COLLECTED)))
                ),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.AUTODROP))),
                sleepms(300),
                execute((() -> targetHeight = 300)),
                sleepms(300),
                execute((() -> claw.setClawState(Claw.clawStates.OPEN))),
//                sleepms(250),
//                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INIT))),
//                sleepms(250),
                sleepms(300),
                parallel(
                        trajectorySequence(preload_to_stack),
                        serial(
                                execute(() -> targetHeight = 80),
                                execute(() -> turret.setTargetAngle(-180)),
                                execute(() -> claw.setSubsystemState(Claw.subsystemStates.READY))
                        )
                ),
                execute((() -> claw.setSubsystemState(Claw.subsystemStates.COLLECTED))),
                sleepms(400),
                execute((() -> targetHeight = 150)),
                sleepms(200),

                // cycle 1

                parallel(
                        execute((() -> targetHeight = 400)),
                        trajectorySequence(cycle1_mid),
                        execute(() -> turret.setTargetAngle(-67))
                ),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.AUTODROP))),
                sleepms(300),
                execute((() -> targetHeight = 300)),
                sleepms(300),
                execute((() -> claw.setClawState(Claw.clawStates.OPEN))),
//                sleepms(200),
//                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INIT))),
//                sleepms(300),
                sleepms(250),
                // high to stack

                parallel(
                        trajectorySequence(cycle1_stack),
                        serial(
                                sleepms(100),
                                execute((() -> targetHeight = 55)),
                                execute((() -> turret.setTargetAngle(-180))),
                                execute((() -> claw.setSubsystemState(Claw.subsystemStates.READY)))
                        )
                ),
                execute((() -> claw.setSubsystemState(Claw.subsystemStates.COLLECTED))),
                sleepms(400),
                execute((() -> targetHeight = 150)),
                sleepms(200),

                // cycle 2

                parallel(
                        trajectorySequence(cycle2_mid),
                        execute((() -> targetHeight = 400)),
                        execute((() -> turret.setTargetAngle(-67)))
                ),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.AUTODROP))),
                sleepms(300),
                execute((() -> targetHeight = 300)),
                sleepms(300),
                execute((() -> claw.setClawState(Claw.clawStates.OPEN))),
//                sleepms(200),
//                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INIT))),
//                sleepms(300),
                sleepms(250),
                // high to stack

                parallel(
                        trajectorySequence(cycle2_stack),
                        serial(
                                sleepms(100),
                                execute((() -> targetHeight = 30)),
                                execute((() -> turret.setTargetAngle(-180))),
                                execute((() -> claw.setSubsystemState(Claw.subsystemStates.READY)))
                        )
                ),
                execute((() -> claw.setSubsystemState(Claw.subsystemStates.COLLECTED))),
                sleepms(400),
                execute((() -> targetHeight = 150)),
                sleepms(200),

                // cycle 3

                parallel(
                        trajectorySequence(cycle3_mid),
                        execute((() -> targetHeight = 400)),
                        execute((() -> turret.setTargetAngle(-67))
                )),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.AUTODROP))),
                sleepms(300),
                execute((() -> targetHeight = 300)),
                sleepms(300),
                execute((() -> claw.setClawState(Claw.clawStates.OPEN))),
//                sleepms(200),
//                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INIT))),
//                sleepms(300),
                sleepms(250),
                // high to stack

                parallel(
                        trajectorySequence(cycle3_stack),
                        serial(
                                sleepms(100),
                                execute((() -> targetHeight = 0)),
                                execute((() -> turret.setTargetAngle(-180))),
                                execute((() -> claw.setSubsystemState(Claw.subsystemStates.READY)))
                        )
                ),
                execute((() -> claw.setSubsystemState(Claw.subsystemStates.COLLECTED))),
                sleepms(400),
                execute((() -> targetHeight = 150)),
                sleepms(200),

                // cycle 4

                parallel(
                        trajectorySequence(cycle4_mid),
                        execute((() -> targetHeight = 400)),
                        execute((() -> turret.setTargetAngle(-67)))
                ),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.AUTODROP))),
                sleepms(300),
                execute((() -> targetHeight = 300)),
                sleepms(300),
                execute((() -> claw.setClawState(Claw.clawStates.OPEN))),
//                sleepms(200),
//                execute((() -> claw.setPivotPosition(Claw.pivotPositions.INIT))),
//                sleepms(300),
                sleepms(250),
                // high to stack

                parallel(
                        trajectorySequence(cycle4_stack),
                        serial(
                                sleepms(100),
                                execute((() -> targetHeight = -10)),
                                execute((() -> turret.setTargetAngle(-180))),
                                execute((() -> claw.setSubsystemState(Claw.subsystemStates.READY)))
                        )
                ),
                execute((() -> claw.setSubsystemState(Claw.subsystemStates.COLLECTED))),
                sleepms(400),
                execute((() -> targetHeight = 150)),
                sleepms(200),

                // cycle 5

                parallel(
                        trajectorySequence(cycle5_mid),
                        serial(
                                sleepms(200),
                        execute((() -> targetHeight = 400)),
                        execute((() -> turret.setTargetAngle(-67)))
                )),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.AUTODROP))),
                sleepms(500),
                execute((() -> targetHeight = 300)),
                execute((() -> claw.setClawState(Claw.clawStates.OPEN))),
                sleepms(1000)
                );
    }
}
