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
    public TrajectorySequence preload_high, preload_to_stack, cycle1_high;

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
                .lineToLinearHeading(new Pose2d(-37.5, -7.5, Math.toRadians(45)))
                .build();

        preload_to_stack = drive.trajectorySequenceBuilder(preload_high.end())
                .lineToLinearHeading(new Pose2d(-55.75, -9))
                .build();

        cycle1_high = drive.trajectorySequenceBuilder(preload_to_stack.end())
                .setAccelConstraint(accelConstraint)
                .setVelConstraint(slowConstraint)
                .lineToLinearHeading(new Pose2d(-35, -12))
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
                        execute((() -> targetHeight = 60)),
                        execute((() -> targetAngle = -180)),
                        execute((() -> claw.setSubsystemState(Claw.subsystemStates.READY)))
                ),
                execute((() -> claw.setSubsystemState(Claw.subsystemStates.COLLECTED))),
                sleepms(200),
                execute((() -> targetHeight = 150)),
                sleepms(200),

                // cycle 1

                parallel(
                        trajectorySequence(cycle1_high),
                        execute((() -> targetHeight = 400)),
                        execute((() -> targetAngle = -60))
                ),
                execute((() -> claw.setPivotPosition(Claw.pivotPositions.DOWN))),
                sleepms(300),
                execute((() -> targetHeight = 300)),
                sleepms(300),
                execute((() -> claw.setClawState(Claw.clawStates.OPEN))),
                sleepms(1000)
                );
    }
}
