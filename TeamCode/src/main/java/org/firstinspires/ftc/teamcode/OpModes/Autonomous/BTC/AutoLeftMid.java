 package org.firstinspires.ftc.teamcode.OpModes.Autonomous.BTC;

import static org.firstinspires.ftc.teamcode.CommonPackage.AutoUtils.pose;
import static org.firstinspires.ftc.teamcode.CommonPackage.AutoUtils.vector;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.execute;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.parallel;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.serial;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.sleepms;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.trajectory;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.trajectorySequence;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.SubSystems.V3.Lift;
import org.firstinspires.ftc.teamcode.SubSystems.V3.Turret;
import org.firstinspires.ftc.teamcode.Tasks.SerialTask;

import java.util.Collections;

@Config
@Autonomous
public class AutoLeftMid extends AutoBase {

    public static double
            PRELOAD_X = -22.7,
            PRELOAD_Y = 33.0,
            STACK1_X = -9.6,
            STACK1_Y = 57.4,
            TH1_X = -20.8,
            TH1_Y = 28.7;
    public Pose2d startPoseLeft = new Pose2d(-62.0, 36, 0.0);
    public TrajectorySequence preloadTrajectory, toStack1,
            toHigh1, toStack4, toHigh2, toHigh3, toHigh4, toStack2, toStack3, toStack5, toHigh5, park1, park2, park3, alignStack;
    TrajectoryVelocityConstraint fastConstraint = new MinVelocityConstraint(Collections.singletonList(
            new TranslationalVelocityConstraint(95)
    ));
    TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(30);

    @Override
    public void onInit() {
        drive.setPoseEstimate(startPoseLeft);
        preloadTrajectory = drive.trajectorySequenceBuilder(startPoseLeft)
                .lineToConstantHeading(vector(
                        PRELOAD_X,
                        PRELOAD_Y
                ))
                .build();
        junction = "B2";

        alignStack = drive.trajectorySequenceBuilder(preloadTrajectory.end())
                .lineToLinearHeading(pose(
                        -10,
                        40,
                        Math.toRadians(-90)
                ))
                .build();

        Turret.fieldSize = 6 * 24;

        toStack1 = drive.trajectorySequenceBuilder(alignStack.end())
                .setAccelConstraint(accelConstraint)
                .lineTo(vector(STACK1_X, STACK1_Y)).build();

        toHigh1 = drive.trajectorySequenceBuilder(toStack1.end())
                .resetConstraints()
                .lineTo(vector(TH1_X + 5.5, TH1_Y)).build();


        toStack2 = drive.trajectorySequenceBuilder(toHigh1.end())
                .setAccelConstraint(accelConstraint)
                .lineTo(vector(STACK1_X - .4, STACK1_Y)).build();

        toHigh2 = drive.trajectorySequenceBuilder(toStack2.end())
                .resetConstraints()
                .lineTo(vector(TH1_X + 4.8, TH1_Y)).build();


        toStack3 = drive.trajectorySequenceBuilder(toHigh2.end())
                .setAccelConstraint(accelConstraint)
                .lineTo(vector(STACK1_X - .6, STACK1_Y)).build();

        toHigh3 = drive.trajectorySequenceBuilder(toStack3.end())
                .resetConstraints()
                .lineTo(vector(TH1_X + 5.0, TH1_Y)).build();


        toStack4 = drive.trajectorySequenceBuilder(toHigh3.end())
                .setAccelConstraint(accelConstraint)
                .lineTo(vector(STACK1_X - .6, STACK1_Y)).build();

        toHigh4 = drive.trajectorySequenceBuilder(toStack4.end())
                .resetConstraints()
                .lineTo(vector(TH1_X + 5.5, TH1_Y)).build();


        toStack5 = drive.trajectorySequenceBuilder(toHigh4.end())
                .setAccelConstraint(accelConstraint)
                .lineTo(vector(STACK1_X - .6, STACK1_Y + .2)).build();

        toHigh5 = drive.trajectorySequenceBuilder(toStack4.end())
                .resetConstraints()
                .lineTo(vector(TH1_X + 5.5, TH1_Y)).build();

        park1 = drive.trajectorySequenceBuilder(toHigh5.end())
                .setVelConstraint(fastConstraint)
                .setAccelConstraint(accelConstraint)
                .lineToConstantHeading(vector(-8, 70))
                .build();

        park2 = drive.trajectorySequenceBuilder(toHigh5.end())
                .setVelConstraint(fastConstraint)
                .setAccelConstraint(accelConstraint)
                .lineToConstantHeading(vector(-8, 33))
                .build();

        park3 = drive.trajectorySequenceBuilder(toHigh5.end())
                .setVelConstraint(fastConstraint)
                .setAccelConstraint(accelConstraint)
                .lineToConstantHeading(vector(-8, 9))
                .build();

        SerialTask drop = serial(
                execute(() -> targetHeight = targetHeight - 330),
                sleepms(150),
                execute(() -> {
                    ct.claw.open();
                    lockOnJunction = false;
                })
        );

        lockOnJunction = true;


        task = serial(
                //preload
                parallel(
                        trajectorySequence(preloadTrajectory),
                        execute(() -> {
                            autoAimOffset = 4;
                            Turret.turretVelocity = .5f;
                            targetHeight = Lift.MID_POS;
                        })
                ),
                execute(() -> {
                    targetHeight = Lift.MID_POS - 950;
                }),
                execute(() -> {
                    ct.claw.open();
                }),
                parallel(
                        trajectorySequence(alignStack),
                        execute(() -> {
                            targetHeight = 520;
                            lockOnJunction = false;
                        }),
                        serial(
                                sleepms(250),
                                execute(() -> {
                                    autoAimOffset = -9;
                                    Turret.turretVelocity = 0.6f;
                                    ct.turret.setTargetAngle(5);
                                }),
                                sleepms(150),
                                execute(() -> {
                                    ct.setCollectPivotAndClawFlip();
                                })
                        )
                ),
                trajectorySequence(toStack1),
                execute(() -> ct.claw.close()),
                sleepms(20),
                execute(() -> {
                    targetHeight = 1030;
                }),
                sleepms(50),
                execute(() -> {
                    ct.pivot.setHalf();
                    lockOnJunction = true;
                    Turret.turretVelocity = .8f;
                }),
                parallel(
                    trajectorySequence(toHigh1),
                    serial(
                            sleepms(360),
                            execute(() -> ct.clawFlip.setScore())
                    )
                ),
                drop,
                // cycle 2
                parallel(
                        trajectorySequence(toStack2),
                        execute(() -> {
                            targetHeight = 450;
                            lockOnJunction = false;
                        }),
                        serial(
                                execute(() -> {
                                    ct.setCollectPivotAndClawFlip();
                                })
                        ),
                        serial(
                                sleepms(300),
                                execute(() -> {
                                    Turret.turretVelocity = 0.6f;
                                    ct.turret.setTargetAngle(2);
                                })
                        )
                ),
                // take from stack
                execute(() -> ct.claw.close()),
                sleepms(20),
                execute(() -> {
                    targetHeight = 1030;
                }),
                sleepms(50),
                execute(() -> {
                    ct.pivot.setHalf();
                    lockOnJunction = true;
                    Turret.turretVelocity = .8f;
                }),
                parallel(
                        trajectorySequence(toHigh2),
                        serial(
                                sleepms(360),
                                execute(() -> ct.clawFlip.setScore())
                        )
                ),
                drop,
                // cycle 3
                parallel(
                        trajectorySequence(toStack3),
                        execute(() -> {
                            targetHeight = 355;
                            lockOnJunction = false;
                        }),
                        serial(
                                execute(() -> {
                                    ct.setCollectPivotAndClawFlip();
                                })
                        ),
                        serial(
                                sleepms(300),
                                execute(() -> {
                                    Turret.turretVelocity = 0.6f;
                                    ct.turret.setTargetAngle(2);
                                })
                        )
                ),
                // take from stack
                execute(() -> ct.claw.close()),
                sleepms(20),
                execute(() -> {
                    targetHeight = 1030;
                }),
                sleepms(50),
                execute(() -> {
                    ct.pivot.setHalf();
                    lockOnJunction = true;
                    Turret.turretVelocity = .8f;
                    autoAimOffset = -10;
                }),
                parallel(
                        trajectorySequence(toHigh3),
                        serial(
                                sleepms(360),
                                execute(() -> ct.clawFlip.setScore())
                        )
                ),
                drop,
                // cycle 4
                parallel(
                        trajectorySequence(toStack4),
                        execute(() -> {
                            targetHeight = 265;
                            lockOnJunction = false;
                        }),
                        serial(
                                execute(() -> {
                                    ct.setCollectPivotAndClawFlip();
                                })
                        ),
                        serial(
                                sleepms(300),
                                execute(() -> {
                                    Turret.turretVelocity = 0.6f;
                                    ct.turret.setTargetAngle(2);
                                })
                        )
                ),
                // take from stack
                execute(() -> ct.claw.close()),
                sleepms(20),
                execute(() -> {
                    targetHeight = 1030;
                }),
                sleepms(50),
                execute(() -> {
                    ct.pivot.setHalf();
                    lockOnJunction = true;
                    Turret.turretVelocity = .8f;
                }),
                parallel(
                        trajectorySequence(toHigh4),
                        serial(
                                sleepms(360),
                                execute(() -> ct.clawFlip.setScore())
                        )
                ),
                drop,
                // cycle 4
                parallel(
                        trajectorySequence(toStack5),
                        execute(() -> {
                            targetHeight = 120;
                            lockOnJunction = false;
                        }),
                        serial(
                                execute(() -> {
                                    ct.setCollectPivotAndClawFlip();
                                })
                        ),
                        serial(
                                sleepms(300),
                                execute(() -> {
                                    Turret.turretVelocity = 0.6f;
                                    ct.turret.setTargetAngle(2);
                                })
                        )
                ),
                // take from stack
                execute(() -> ct.claw.close()),
                sleepms(20),
                execute(() -> {
                    targetHeight = 1030;
                }),
                sleepms(50),
                execute(() -> {
                    ct.pivot.setHalf();
                    lockOnJunction = true;
                    Turret.turretVelocity = .8f;
                }),
                parallel(
                        trajectorySequence(toHigh5),
                        serial(
                                sleepms(360),
                                execute(() -> ct.clawFlip.setScore())
                        )
                ),
                drop,
                parallel(
                    trajectorySequence(park3),
                        serial(
                                sleepms(10),
                                execute(() -> {
                                    targetHeight = 0;
                                    ct.turret.setTargetAngle(0);
                                })
                        )
                )
        );

    }
}

