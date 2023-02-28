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
@Autonomous(name = "Auto")
@Config
public class Auto extends AutoBase {
    public Pose2d startPoseLeft;
    public TrajectorySequence preload_high, preload_to_stack, cycle1_high, cycle2_high, cycle3_high, cycle4_high, cycle5_high;

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
                .lineToLinearHeading(new Pose2d(-55.4, -8.75))
                .build();

        cycle1_high = drive.trajectorySequenceBuilder(preload_to_stack.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(-30.5, -11))
                .resetConstraints()
                .build();

        cycle2_high = drive.trajectorySequenceBuilder(preload_to_stack.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(-30.5, -11))
                .resetConstraints()
                .build();

        cycle3_high = drive.trajectorySequenceBuilder(preload_to_stack.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(-30.5, -11.5))
                .resetConstraints()
                .build();

        cycle4_high = drive.trajectorySequenceBuilder(preload_to_stack.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(-30.5, -12.2))
                .resetConstraints()
                .build();

        cycle5_high = drive.trajectorySequenceBuilder(preload_to_stack.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(-30.5, -12.2))
                .resetConstraints()
                .build();



        task = serial(
                parallel(
                        trajectorySequence(preload_high),
                        execute((() -> targetHeight = 400)),
                        execute((() -> claw.setSubsystemState(Claw.subsystemStates.COLLECTED)))
                ),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.DOWN))),
                sleepms(300),
                execute((() -> targetHeight = 300)),
                sleepms(300),
                execute((() -> claw.setClawState(Claw.clawStates.OPEN))),
                sleepms(200),

                parallel(
                        trajectorySequence(preload_to_stack),
                        serial(
//                                sleepms(50),
                                execute((() -> targetHeight = 65)),
                                execute((() -> targetAngle = -180)),
                                execute((() -> claw.setSubsystemState(Claw.subsystemStates.READY)))
                        )
                ),
                execute((() -> claw.setSubsystemState(Claw.subsystemStates.COLLECTED))),
                sleepms(200),
                execute((() -> targetHeight = 150)),
                sleepms(200),

                // cycle 1

                parallel(
                        execute((() -> targetHeight = 400)),
                        trajectorySequence(cycle1_high),
                        execute((() -> targetAngle = -67))
                ),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.DOWN))),
                sleepms(300),
                execute((() -> targetHeight = 300)),
                sleepms(300),
                execute((() -> claw.setClawState(Claw.clawStates.OPEN))),
                sleepms(500),

                // high to stack

                parallel(
                        trajectorySequence(preload_to_stack),
                        serial(
//                                sleepms(50),
                                execute((() -> targetHeight = 51)),
                                execute((() -> targetAngle = -180)),
                                execute((() -> claw.setSubsystemState(Claw.subsystemStates.READY)))
                        )
                ),
                execute((() -> claw.setSubsystemState(Claw.subsystemStates.COLLECTED))),
                sleepms(200),
                execute((() -> targetHeight = 150)),
                sleepms(200),

                // cycle 2

                parallel(
                        trajectorySequence(cycle2_high),
                        execute((() -> targetHeight = 400)),
                        execute((() -> targetAngle = -67))
                ),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.DOWN))),
                sleepms(300),
                execute((() -> targetHeight = 300)),
                sleepms(300),
                execute((() -> claw.setClawState(Claw.clawStates.OPEN))),
                sleepms(500),

                // high to stack

                parallel(
                        trajectorySequence(preload_to_stack),
                        serial(
//                                sleepms(50),
                                execute((() -> targetHeight = 36)),
                                execute((() -> targetAngle = -180)),
                                execute((() -> claw.setSubsystemState(Claw.subsystemStates.READY)))
                        )
                ),
                execute((() -> claw.setSubsystemState(Claw.subsystemStates.COLLECTED))),
                sleepms(200),
                execute((() -> targetHeight = 150)),
                sleepms(200),

                // cycle 3

                parallel(
                        trajectorySequence(cycle3_high),
                        execute((() -> targetHeight = 400)),
                        execute((() -> targetAngle = -67))
                ),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.DOWN))),
                sleepms(300),
                execute((() -> targetHeight = 300)),
                sleepms(300),
                execute((() -> claw.setClawState(Claw.clawStates.OPEN))),
                sleepms(500),

                // high to stack

                parallel(
                        trajectorySequence(preload_to_stack),
                        serial(
//                                sleepms(50),
                                execute((() -> targetHeight = 15)),
                                execute((() -> targetAngle = -180)),
                                execute((() -> claw.setSubsystemState(Claw.subsystemStates.READY)))
                        )
                ),
                execute((() -> claw.setSubsystemState(Claw.subsystemStates.COLLECTED))),
                sleepms(200),
                execute((() -> targetHeight = 150)),
                sleepms(200),

                // cycle 4

                parallel(
                        trajectorySequence(cycle4_high),
                        execute((() -> targetHeight = 400)),
                        execute((() -> targetAngle = -67))
                ),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.DOWN))),
                sleepms(300),
                execute((() -> targetHeight = 300)),
                sleepms(300),
                execute((() -> claw.setClawState(Claw.clawStates.OPEN))),
                sleepms(500),

                // high to stack

                parallel(
                        trajectorySequence(preload_to_stack),
                        serial(
//                                sleepms(50),
                                execute((() -> targetHeight = 0)),
                                execute((() -> targetAngle = -180)),
                                execute((() -> claw.setSubsystemState(Claw.subsystemStates.READY)))
                        )
                ),
                execute((() -> claw.setSubsystemState(Claw.subsystemStates.COLLECTED))),
                sleepms(200),
                execute((() -> targetHeight = 150)),
                sleepms(200),

                // cycle 5

                parallel(
                        trajectorySequence(cycle5_high),
                        execute((() -> targetHeight = 400)),
                        execute((() -> targetAngle = -67))
                ),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.DOWN))),
                sleepms(300),
                execute((() -> targetHeight = 300)),
                sleepms(300),
                execute((() -> claw.setClawState(Claw.clawStates.OPEN))),
                sleepms(300)
        );
    }
}
