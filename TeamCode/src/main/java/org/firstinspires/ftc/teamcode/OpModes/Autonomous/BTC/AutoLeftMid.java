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
import com.acmerobotics.roadrunner.profile.AccelerationConstraint;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MinAccelerationConstraint;
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
            PRELOAD_X = -22.5,
            PRELOAD_Y = 31.0,
            PRELOAD_HEADING = Math.toRadians(-60.5),
            STACK_ALIGN_X = -8.7,
            STACK_ALIGN_Y = 46.9,
            STACK_ALIGN_HEADING = Math.toRadians(-90.0),
            STACK1_X = -8.0,
            STACK1_Y = 59.7,
            TH1_X = -20.5,
            TH1_Y = 30.8;

    TrajectoryVelocityConstraint fastConstraint = new MinVelocityConstraint(Collections.singletonList(
            new TranslationalVelocityConstraint(95)
    ));
    TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(37);
    public Pose2d startPoseLeft = new Pose2d(-62.0, 36, 0.0);
    public TrajectorySequence preloadTrajectory, toStackAlign, toStackAlign2, toStack1,
            toHigh1, toStack4, toStackAlign4, toStackAlign3, toHigh2, toHigh3, toHigh4, toStack2, toStack3, toStack5, toStack6, toStackAlign6, toStackAlign5, toHigh5, toHigh6, park1;

    @Override
    public void onInit() {
        drive.setPoseEstimate(startPoseLeft);
        preloadTrajectory = drive.trajectorySequenceBuilder(startPoseLeft)
                .lineToLinearHeading(pose(
                        PRELOAD_X,
                        PRELOAD_Y,
                        PRELOAD_HEADING
                ))
                .build();
        junction = "B2";
        autoAimOffset = -3;

        Turret.fieldSize = 6 * 24;

        toStackAlign = drive.trajectorySequenceBuilder(preloadTrajectory.end())
                .resetConstraints()
                .lineToLinearHeading(pose(STACK_ALIGN_X, STACK_ALIGN_Y, STACK_ALIGN_HEADING))
                .build();

        toStack1 = drive.trajectorySequenceBuilder(toStackAlign.end())
                .setAccelConstraint(accelConstraint)
                .lineTo(vector(STACK1_X, STACK1_Y - 2.5)).build();

        toHigh1 = drive.trajectorySequenceBuilder(toStack1.end())
                .resetConstraints()
                .lineTo(vector(TH1_X + 5.5, TH1_Y)).build();

        toStackAlign2 = drive.trajectorySequenceBuilder(toHigh1.end())
                .resetConstraints()
                .lineTo(vector(-12.0, 36.0)).build();

        toStack2 = drive.trajectorySequenceBuilder(toStackAlign2.end())
                .setAccelConstraint(accelConstraint)
                .lineTo(vector(STACK1_X, STACK1_Y - 2.5)).build();

        toHigh2 = drive.trajectorySequenceBuilder(toStack2.end())
                .resetConstraints()
                .lineTo(vector(TH1_X + 4.8, TH1_Y)).build();

        toStackAlign3 = drive.trajectorySequenceBuilder(toHigh2.end())
                .resetConstraints()
                .lineTo(vector(-12.0, 36.0)).build();

        toStack3 = drive.trajectorySequenceBuilder(toStackAlign3.end())
                .setAccelConstraint(accelConstraint)
                .lineTo(vector(STACK1_X + 1, STACK1_Y - 2.5)).build();

        toHigh3 = drive.trajectorySequenceBuilder(toStack3.end())
                .resetConstraints()
                .lineTo(vector(TH1_X + 5.0, TH1_Y)).build();

        toStackAlign4 = drive.trajectorySequenceBuilder(toHigh3.end())
                .resetConstraints()
                .lineTo(vector(-12.0, 36.0)).build();

        toStack4 = drive.trajectorySequenceBuilder(toStackAlign4.end())
                .setAccelConstraint(accelConstraint)
                .lineTo(vector(STACK1_X, STACK1_Y - 2.5)).build();

        toHigh4 = drive.trajectorySequenceBuilder(toStack4.end())
                .resetConstraints()
                .lineTo(vector(TH1_X + 5.5, TH1_Y)).build();

        toStackAlign5 = drive.trajectorySequenceBuilder(toHigh3.end())
                .resetConstraints()
                .lineTo(vector(-12.0, 36.0)).build();

        toStack5 = drive.trajectorySequenceBuilder(toStackAlign4.end())
                .setAccelConstraint(accelConstraint)
                .lineTo(vector(STACK1_X, STACK1_Y - 2.5)).build();

        toHigh5 = drive.trajectorySequenceBuilder(toStack4.end())
                .resetConstraints()
                .lineTo(vector(TH1_X + 5.5, TH1_Y)).build();

        toStackAlign6 = drive.trajectorySequenceBuilder(toHigh3.end())
                .resetConstraints()
                .lineTo(vector(-12.0, 36.0)).build();

        toStack6 = drive.trajectorySequenceBuilder(toStackAlign4.end())
                .lineTo(vector(STACK1_X, STACK1_Y - 2.5)).build();

        toHigh6 = drive.trajectorySequenceBuilder(toStack4.end())
                .lineTo(vector(TH1_X + 5.5, TH1_Y)).build();

        park1 = drive.trajectorySequenceBuilder(toHigh5.end())
                .setVelConstraint(fastConstraint)
                .setAccelConstraint(accelConstraint)
                .lineToConstantHeading(vector(-8, 70))
                .build();

        lockOnJunction = true;

        SerialTask drop = serial(
                execute(() -> targetHeight = targetHeight - 500),
                sleepms(150),
                execute(() -> {
                    ct.claw.open();
                    lockOnJunction = false;
                })
        );


        task = serial(
                //preload
                parallel(
                        trajectorySequence(preloadTrajectory),
                        execute(() -> {
                            targetHeight = Lift.MID_POS;
                        })
                ),
                execute(() -> {
                    targetHeight = Lift.MID_POS - 950;
                }),
                execute(() -> {
                    ct.claw.open();
                }),
                
                serial(
                        parallel(trajectorySequence(toStackAlign),
                                execute(() -> {
                                    targetHeight = 520;
                                }),
                                serial(
                                        sleepms(250),
                                        execute(() -> {
                                            ct.setCollectPivotAndClawFlip();
                                        })
                                )),
                        //cycle 1
                        execute(() -> {
                            lockOnJunction = false;
                            Turret.turretVelocity = 1f;
                            ct.turret.setTargetAngle(0);
                        }),
                        trajectorySequence(toStack1),
                        execute(() -> ct.claw.close()),
                        sleepms(20),
                        execute(() -> {
                            targetHeight = 960;
                        }),
                        sleepms(50),
                        execute(() -> {
                            ct.pivot.setHalf();
                        })
                ),
                parallel(
                        trajectorySequence(toHigh1),
                        execute(() -> {
                            targetHeight = Lift.MID_POS;
                            Turret.turretVelocity = .5;
                            lockOnJunction = true;
                            junction = "B2";
                        }),
                        serial(
                                sleepms(300),
                                execute(() -> {
                                    ct.clawFlip.setScore();
                                }))
                ),
                drop,
                serial(
                        parallel(
                                trajectorySequence(toStackAlign2),
                                execute(() -> {
                                    targetHeight = 395;
                                }),
                                serial(
                                        sleepms(300),
                                        execute(() -> {
                                            ct.setCollectPivotAndClawFlip();
                                        })
                                )),
                        //cycle 1
                        execute(() -> {
                            Turret.turretVelocity = 1f;
                            ct.turret.setTargetAngle(0);
                        }),
                        trajectorySequence(toStack2),
                        execute(() -> ct.claw.close()),
                        sleepms(20),
                        execute(() -> {
                            targetHeight = 960;
                        }),
                        sleepms(50),
                        execute(() -> {
                            ct.pivot.setHalf();
                        })
                ),
                parallel(
                        trajectorySequence(toHigh2),
                        execute(() -> {
                            autoAimOffset = -2;
                            targetHeight = Lift.MID_POS;
                            lockOnJunction = true;
                            Turret.turretVelocity = .6;
                            junction = "B2";
                        }),
                        serial(
                                sleepms(300),
                                execute(() -> {
                                    ct.clawFlip.setScore();
                                }))
                ),
                drop,
                serial(
                        parallel(trajectorySequence(toStackAlign3),
                                execute(() -> {
                                    targetHeight = 360;
                                }),
                                serial(
                                        sleepms(300),
                                        execute(() -> {
                                            ct.setCollectPivotAndClawFlip();
                                        })
                                )),
                        //cycle 1
                        execute(() -> {
                            Turret.turretVelocity = 1f;
                            ct.turret.setTargetAngle(0);
                        }),
                        trajectorySequence(toStack3),
                        execute(() -> ct.claw.close()),
                        sleepms(20),
                        execute(() -> {
                            targetHeight = 960;
                        }),
                        sleepms(50),
                        execute(() -> {
                            ct.pivot.setHalf();
                        })
                ),
                parallel(
                        trajectorySequence(toHigh3),
                        execute(() -> {
                            targetHeight = Lift.MID_POS;
                            Turret.turretVelocity = .5;
                            lockOnJunction = true;
                            junction = "B2";
                        }),
                        serial(
                                sleepms(300),
                                execute(() -> {
                                    ct.clawFlip.setScore();
                                }))
                ),
                drop,
                serial(
                        parallel(trajectorySequence(toStackAlign4),
                                execute(() -> {
                                    targetHeight = 230;
                                }),
                                serial(
                                        sleepms(200),
                                        execute(() -> {
                                            ct.setCollectPivotAndClawFlip();
                                        })
                                )),
                        //cycle 1
                        execute(() -> {
                            Turret.turretVelocity = 1f;
                            ct.turret.setTargetAngle(0);
                        }),
                        trajectorySequence(toStack4),
                        execute(() -> ct.claw.close()),
                        sleepms(20),
                        execute(() -> {
                            targetHeight = 960;
                        }),
                        sleepms(50),
                        execute(() -> {
                            ct.pivot.setHalf();
                        })
                ),
                parallel(
                        trajectorySequence(toHigh4),
                        execute(() -> {
//                            autoAimOffset = -7;
                            targetHeight = Lift.MID_POS;
                            Turret.turretVelocity = .5;
                            lockOnJunction = true;
                            junction = "B2";
                        }),
                        serial(
                                sleepms(300),
                                execute(() -> {
                                    ct.clawFlip.setScore();
                                }))
                ),
                drop,
                serial(
                        parallel(trajectorySequence(toStackAlign5),
                                execute(() -> {
                                    targetHeight = 50;
                                }),
                                serial(
                                        sleepms(300),
                                        execute(() -> {
                                            ct.setCollectPivotAndClawFlip();
                                        })
                                )),
                        //cycle 1
                        execute(() -> {
                            Turret.turretVelocity = 1f;
                            ct.turret.setTargetAngle(0);
                        }),
                        trajectorySequence(toStack5),
                        execute(() -> ct.claw.close()),
                        sleepms(20),
                        execute(() -> {
                            targetHeight = 960;
                        }),
                        sleepms(50),
                        execute(() -> {
                            ct.pivot.setHalf();
                        })
                ),
                parallel(
                        trajectorySequence(toHigh5),
                        execute(() -> {
                            targetHeight = Lift.MID_POS;
                            Turret.turretVelocity = .5;
                            lockOnJunction = true;
                            junction = "B2";
                        }),
                        serial(
                                sleepms(300),
                                execute(() -> {
                                    ct.clawFlip.setScore();
                                }))
                ),
                drop,
                trajectorySequence(park1),
                execute(() -> {
                    Turret.turretVelocity = 0;
                    ct.turret.setTargetAngle(0);
                    targetHeight = 0;
                })
        );

    }
}

