package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.geometry.*;
import com.acmerobotics.roadrunner.trajectory.constraints.*;

import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.SubSystems.V2.Claw;

import java.util.Arrays;

//@Disabled
@Autonomous(name = "Auto High")
@Config
public class AutoHigh extends AutoBase {
    public Pose2d startPoseLeft;
    public TrajectorySequence preload_high, preload_to_stack;
    public TrajectorySequence cycle1_high, cycle2_high, cycle3_high, cycle4_high, cycle5_high;
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
                .lineToLinearHeading(new Pose2d(-35.8, -6.5, Math.toRadians(45)))
                .build();

        preload_to_stack = drive.trajectorySequenceBuilder(preload_high.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(-56, -8.75))
                .build();

        cycle1_high = drive.trajectorySequenceBuilder(preload_to_stack.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(-30, -12.5))
                .resetConstraints()
                .build();

        cycle1_stack = drive.trajectorySequenceBuilder(cycle1_high.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(-56, -8.75))
                .build();

        cycle2_high = drive.trajectorySequenceBuilder(cycle1_stack.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(-30, -12.5))
                .resetConstraints()
                .build();

        cycle2_stack = drive.trajectorySequenceBuilder(cycle2_high.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(-56.25, -9.25))
                .build();

        cycle3_high = drive.trajectorySequenceBuilder(cycle2_stack.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(-30, -12.5))
                .resetConstraints()
                .build();

        cycle3_stack = drive.trajectorySequenceBuilder(cycle3_high.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(-56.25, -9.75))
                .build();

        cycle4_high = drive.trajectorySequenceBuilder(cycle3_stack.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(-30, -13.2))
                .resetConstraints()
                .build();

        cycle4_stack = drive.trajectorySequenceBuilder(cycle4_high.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(-56.25, -9.75))
                .build();


        cycle5_high = drive.trajectorySequenceBuilder(cycle4_stack.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(-30, -13.2))
                .resetConstraints()
                .build();

        zone1 = drive.trajectorySequenceBuilder(cycle5_high.end())
                .lineToLinearHeading(new Pose2d(-56.25, -10))
                .build();

        zone2 = drive.trajectorySequenceBuilder(cycle5_high.end())
                .lineToLinearHeading(new Pose2d(-30, -10))
                .build();

        zone3 = drive.trajectorySequenceBuilder(cycle5_high.end())
                .lineToLinearHeading(new Pose2d(-10, -10))
                .build();

        task = serial(
                parallel(
                        trajectorySequence(preload_high),
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
                        trajectorySequence(cycle1_high),
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
                        trajectorySequence(cycle2_high),
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
                        trajectorySequence(cycle3_high),
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
                        trajectorySequence(cycle4_high),
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
                        trajectorySequence(cycle5_high),
                        serial(
                                sleepms(200),
                        execute((() -> targetHeight = 400)),
                        execute((() -> turret.setTargetAngle(-67)))
                )),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.AUTODROP))),
                sleepms(300),
                execute((() -> targetHeight = 300)),
                execute((() -> claw.setClawState(Claw.clawStates.OPEN))),
                execute(() -> {
                    switch(preferredZone){
                        case 1:
                            claw.setSubsystemState(Claw.subsystemStates.READY);
                            turret.setTargetAngle(0.0);
                            targetHeight = 0;
                            drive.followTrajectorySequence(zone1);
                            break;
                        case 2:
                            claw.setSubsystemState(Claw.subsystemStates.READY);
                            turret.setTargetAngle(0.0);
                            targetHeight = 0;
                            drive.followTrajectorySequence(zone2);
                            break;
                        case 3:
                            claw.setSubsystemState(Claw.subsystemStates.READY);
                            turret.setTargetAngle(0.0);
                            targetHeight = 0;
                            drive.followTrajectorySequence(zone3);
                            break;
                    }
                })
                );
    }
}
