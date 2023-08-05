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
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.SubSystems.V3.Lift;

import java.util.Collections;

@Config
@Disabled
@Autonomous
public class AutoTransformer extends AutoBase {
    public static double
            START_TO_HIGH_X_1 = -26,
            START_TO_HIGH_Y_1 = 55;
    public Pose2d startPoseCenter = new Pose2d(-88.27, 59.5);
    public TrajectorySequence preload, alignStraight, straight, alignCollect, collect, alignStraightBack, alignBack, placeTransformer, alignPark, park1, park2;
    TrajectoryVelocityConstraint fastConstraint = new MinVelocityConstraint(Collections.singletonList(
            new TranslationalVelocityConstraint(70)
    ));
    ProfileAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(40);
//    AngularVelocityConstraint angularVelocityConstraint = new AngularVelocityConstraint(100);

    @Override
    public void onInit() {
        drive.setPoseEstimate(startPoseCenter);

        preload = drive.trajectorySequenceBuilder(startPoseCenter)
                .lineToLinearHeading(pose(
                        START_TO_HIGH_X_1,
                        START_TO_HIGH_Y_1,
                        Math.toRadians(-90)
                ))
                .build();

        alignStraight = drive.trajectorySequenceBuilder(preload.end())
                .setReversed(true)
                .lineToConstantHeading(vector(
                        -12,
                        59.5
                ))
                .build();

        straight = drive.trajectorySequenceBuilder(alignStraight.end())
                .setVelConstraint(fastConstraint)
                .setAccelConstraint(accelConstraint)
                .setReversed(true)
                .lineToConstantHeading(vector(
                        -12,
                        59.5 + 24 + 24
                ))
                .build();

        alignCollect = drive.trajectorySequenceBuilder(straight.end())
                .resetConstraints()
                .setReversed(true)
                .lineToConstantHeading(vector(
                        0,
                        59.5 + 24 + 24
                ))
                .build();

        collect = drive.trajectorySequenceBuilder(alignCollect.end())
                .setReversed(true)
                .lineToConstantHeading(vector(
                        -0.5,
                        81.5 + 24 + 24
                ))
                .build();

        alignStraightBack = drive.trajectorySequenceBuilder(collect.end())
                .setReversed(true)
                .lineToConstantHeading(straight.end().vec())
                .build();

        alignBack = drive.trajectorySequenceBuilder(alignStraightBack.end())
                .setVelConstraint(fastConstraint)
                .setAccelConstraint(accelConstraint)
                .lineToConstantHeading(alignStraight.end().vec())
                .build();

        placeTransformer = drive.trajectorySequenceBuilder(alignBack.end())
                .resetConstraints()
                .lineToLinearHeading(new Pose2d(-88.27 + 24 + 12, 63.5, Math.toRadians(0)))
                .build();

        alignPark = drive.trajectorySequenceBuilder(placeTransformer.end())
                .lineToConstantHeading(vector(-14, 63.5))
                .build();


        park1 = drive.trajectorySequenceBuilder(alignPark.end())
                .resetConstraints()
                .lineToConstantHeading(vector(-14, 63.5 - 24))
                .build();

        park2 = drive.trajectorySequenceBuilder(alignPark.end())
                .resetConstraints()
                .lineToConstantHeading(vector(-14, 63.5 + 24))
                .build();


        ct.claw.close();

        task = serial(
                parallel(
                        trajectorySequence(preload),
                        serial(
                                execute(() -> {
                                    ct.turret.setTurretVelocity(1.0);
                                    ct.turret.setTargetAngle(30);
                                }),
                                sleepms(201),
                                execute(() -> {
                                    targetHeight = Lift.HIGH_POS;
                                    ct.pivot.setScore();
                                })
                        )
                ),
                sleepms(70),
                execute(() -> {
                    targetHeight = Lift.HIGH_POS - 100 - 600;
                }),
                sleepms(50),
                execute(() -> {
                    ct.claw.open();
                }),
                sleepms(42),
                parallel(
                        execute(() -> {
                            ct.turret.setTurretVelocity(1.0);
                            ct.turret.setTargetAngle(0);
                            ct.clawFlip.setScore();
                            ct.claw.open();
                            targetHeight = 50;
                        })
                ),
                trajectorySequence(alignStraight),
                trajectorySequence(straight),
                execute(() -> {
                    ct.setCollectPivotAndClawFlip();
                }),
                trajectorySequence(alignCollect),
                parallel(
                        trajectorySequence(collect),
                        serial(
                                sleepms(1500),
                                execute(() -> ct.claw.close())
                        )
                ),
                sleepms(450),
                execute(() -> ct.pivot.setScore()),
                trajectorySequence(alignStraightBack),
                trajectorySequence(alignBack),
                parallel(
                        trajectorySequence(placeTransformer),
                        serial(
                                sleepms(2000),
                                execute(() -> ct.setCollectPivotAndClawFlip())
                        )
                ),
                sleepms(1000),
                execute(() -> ct.claw.open()),
                parallel(
                        serial(
                                sleepms(500),
                                execute(() -> ct.setScorePivotAndClawFlip())
                        ),
                        trajectorySequence(alignPark)
                ),
                execute(() -> {
                    if (preferredZone == 1) {
                        drive.followTrajectorySequence(park1);
                    } else if (preferredZone == 2) {
                        drive.followTrajectorySequence(park2);
                    }
                })


        );
    }
}
