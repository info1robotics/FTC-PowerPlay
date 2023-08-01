package org.firstinspires.ftc.teamcode.OpModes.Autonomous.beclean;

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

import org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.SubSystems.V3.Lift;
import org.firstinspires.ftc.teamcode.Tasks.SerialTask;

import java.util.Collections;

@Config
@Autonomous
public class AutoLeftUnscore extends AutoBase {
    public static int HIGH_OFFSET = -970;
    public static int CONE_OFFSET = 112;

    public static double
            HIGH_X_1 = -53.74,
            HIGH_Y_1 = 32.5,
            HIGH_HEADING = -45;

    public Pose2d startPose = new Pose2d(-88.27, 59.5 + 24);

    public TrajectorySequence trajectoryHigh, trajectoryHighTwo;
    TrajectoryVelocityConstraint fastConstraint = new MinVelocityConstraint(Collections.singletonList(
            new TranslationalVelocityConstraint(120)
    ));
    ProfileAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(120);

    public TrajectorySequence repositionTrajectory() {
        return drive.trajectorySequenceBuilder(drive.getLocalizer().getPoseEstimate())
                .lineToLinearHeading(pose(
                        HIGH_X_1,
                        HIGH_Y_1,
                        Math.toRadians(HIGH_HEADING)
                ))
                .build();
    }

    @Override
    public void onInit() {
        drive.setPoseEstimate(startPose);

        trajectoryHigh = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(vector(
                        -86.0,
                        35
                ))
                .addSpatialMarker(vector(
                                -86.0,
                                35
                        ), () -> {
                            targetHeight = Lift.HIGH_POS + HIGH_OFFSET;
                            lockOnJunction = false;
                            junction = "C2";
                            ct.turret.setTargetAngle(-6);
                        }
                )
                .lineToLinearHeading(pose(
                        HIGH_X_1,
                        HIGH_Y_1,
                        Math.toRadians(HIGH_HEADING)
                ))
                .build();

        trajectoryHighTwo = drive.trajectorySequenceBuilder(trajectoryHigh.end())
                .back(5)
                .lineToConstantHeading(vector(
                        -5.27,
                        38
                ))
                .lineToConstantHeading(vector(
                        -6.8,
                        31.8
                ))
                .addSpatialMarker(vector(-7.8, 31), () -> {
                    new Thread(() -> {
                        sleep(1400);
                    ct.claw.close();
                    }).start();

                })
                .build();


        ct.setScorePivotAndClawFlip();
        ct.claw.open();
        ct.claw.open();

        SerialTask launch = serial(
                execute(() -> {
                    ct.pivot.setHalf();
                    ct.clawFlip.setCollect();
                }),
                sleepms(100),
                execute(() -> {
                    ct.claw.open();
                })
        );

        task = serial(
                trajectorySequence(trajectoryHigh),
                execute(() -> {
                    ct.claw.close();
                }),
                sleepms(200),
                execute(() -> {
                    targetHeight = Lift.HIGH_POS - 100;
                }),
                sleepms(200),
                launch,
                execute(() -> {
                    for (int i = 1; i <= 4; i++) {
                        ct.setScorePivotAndClawFlip();
                        sleep(300);
                        ct.lift.setHeight(Lift.HIGH_POS + HIGH_OFFSET - CONE_OFFSET * i + 40, linkageVelocity);
                        sleep(600 + i * 140);
                        ct.claw.close();
                        sleep(200);
                        ct.lift.setHeight(Lift.HIGH_POS - 100, linkageVelocity);
                        sleep(500 + i * 140);
                        ct.pivot.setHalf();
                        ct.clawFlip.setCollect();
                        sleep(100);
                        ct.claw.open();
                    }
                    for (int i = 5; i <= 11; i++) {
                        ct.setScorePivotAndClawFlip();
                        sleep(300);
                        ct.lift.setHeight(Lift.HIGH_POS + HIGH_OFFSET - CONE_OFFSET * (i - 2) - 20, linkageVelocity);
                        sleep(600 + i * 60);
                        ct.claw.close();
                        sleep(200);
                        ct.lift.setHeight(Lift.HIGH_POS - 100, linkageVelocity);
                        sleep(500 + i * 88);
                        ct.pivot.setHalf();
                        ct.clawFlip.setCollect();
                        sleep(100);
                        ct.claw.open();
                    }
                }),
                parallel(
                        trajectorySequence(trajectoryHighTwo),
                        serial(
                                execute(() -> {
                                    ct.setScorePivotAndClawFlip();
                                    lockOnJunction = false;
                                    junction = "C4";
//                                    autoAimOffset = 14;
                                }),
                                sleepms(300),
                                execute(() -> {
                                    targetHeight = 0;
                                }),
                                sleepms(400),
                                execute(() -> {
                                    lockOnJunction = true;
                                }),
                                sleepms(1000),
                                execute(() -> {
                                    targetHeight = Lift.HIGH_POS + HIGH_OFFSET;
                                })
                        )
                ),
                execute(() -> {
                    ct.claw.close();
                }),
                sleepms(200),
                execute(() -> {
                    targetHeight = Lift.HIGH_POS - 100;
                }),
                sleepms(200),
                launch,
                execute(() -> {
                    for (int i = 1; i <= 4; i++) {
                        ct.setScorePivotAndClawFlip();
                        sleep(300);
                        ct.lift.setHeight(Lift.HIGH_POS + HIGH_OFFSET - CONE_OFFSET * i, linkageVelocity);
                        sleep(600 + i * 140);
                        ct.claw.close();
                        sleep(200);
                        ct.lift.setHeight(Lift.HIGH_POS - 100, linkageVelocity);
                        sleep(500 + i * 140);
                        ct.pivot.setHalf();
                        ct.clawFlip.setCollect();
                        sleep(100);
                        ct.claw.open();
                    }
                    for (int i = 5; i <= 11; i++) {
                        ct.setScorePivotAndClawFlip();
                        sleep(300);
                        ct.lift.setHeight(Lift.HIGH_POS + HIGH_OFFSET - CONE_OFFSET * (i - 2) - 20, linkageVelocity);
                        sleep(600 + i * 60);
                        ct.claw.close();
                        sleep(200);
                        ct.lift.setHeight(Lift.HIGH_POS - 100, linkageVelocity);
                        sleep(500 + i * 88);
                        ct.pivot.setHalf();
                        ct.clawFlip.setCollect();
                        sleep(100);
                        ct.claw.open();
                    }
                })
        );
    }
}
