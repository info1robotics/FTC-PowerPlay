package org.firstinspires.ftc.teamcode.OpModes.Autonomous.CRI;

import static org.firstinspires.ftc.teamcode.CommonPackage.AutoUtils.pose;
import static org.firstinspires.ftc.teamcode.CommonPackage.AutoUtils.vector;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.execute;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.parallel;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.serial;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.sleepms;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.trajectorySequence;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.SubSystems.V3.Lift;

import java.util.Collections;

@Config
@Autonomous
public class AutoTransformer extends AutoBase {
    public Pose2d startPoseCenter = new Pose2d(-88.27, 11.5);
    public TrajectorySequence forward, align, transformerAlign, transformer;


    TrajectoryVelocityConstraint fastConstraint = new MinVelocityConstraint(Collections.singletonList(
            new TranslationalVelocityConstraint(120)
    ));
    ProfileAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(70);
    AngularVelocityConstraint angularVelocityConstraint = new AngularVelocityConstraint(100);

    @Override
    public void onInit() {
        drive.setPoseEstimate(startPoseCenter);

        forward = drive.trajectorySequenceBuilder(startPoseCenter)
                .setVelConstraint(fastConstraint)
                .setAccelConstraint(accelConstraint)
                .lineToConstantHeading(vector(-9, 18))
                .addSpatialMarker(
                        vector(-43, 15),
                        () -> {
                            targetHeight = Lift.HIGH_POS;
                        }
                )
                .addSpatialMarker(
                        vector(-26, 15),
                        () -> {
                            ct.turret.setTargetAngle(30);
                        }
                )
                .build();

        align = drive.trajectorySequenceBuilder(forward.end())
                .setTurnConstraint(100, 70)
                .lineToLinearHeading(pose(-12, 12, Math.toRadians(90)))
                .build();

        transformerAlign = drive.trajectorySequenceBuilder(align.end())
                .setVelConstraint(fastConstraint)
                .setAccelConstraint(accelConstraint)
                .lineToLinearHeading(pose(-12, 70.5, Math.toRadians(90)))
                .addSpatialMarker(
                        vector(-12, 30),
                        () -> {
                            ct.lift.setHeight(100, 1.0);
                            ct.turret.setTargetAngle(180 - 19);
                        }
                )
                .addSpatialMarker(
                        vector(-12, 55),
                        () -> {
                            ct.setCollectPivotAndClawFlip();
                        }
                )
                .build();

        transformer = drive.trajectorySequenceBuilder(transformerAlign.end())
                .resetConstraints()
                .lineToConstantHeading(vector(-8.8, 75.5))
                .build();


        ct.claw.close();

        task = serial(
                trajectorySequence(forward),
                sleepms(1200),
                execute(() -> targetHeight = Lift.HIGH_POS - 600),
                sleepms(100),
                execute(() -> ct.claw.open()),
                parallel(
                        trajectorySequence(align),
                        execute(() -> {
                            ct.turret.setTargetAngle(180);
                            targetHeight = 100;
                        })
                ),
                execute(() -> {
                    ct.claw.open();
                }),
                trajectorySequence(transformerAlign),
                sleepms(1000),
                trajectorySequence(transformer),
                execute(() -> {
                    ct.setScorePivotAndClawFlip();
                })
        );
    }
}
