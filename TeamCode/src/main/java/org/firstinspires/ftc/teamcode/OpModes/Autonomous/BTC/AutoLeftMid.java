package org.firstinspires.ftc.teamcode.OpModes.Autonomous.BTC;

import static org.firstinspires.ftc.teamcode.CommonPackage.AutoUtils.pose;
import static org.firstinspires.ftc.teamcode.CommonPackage.AutoUtils.vector;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.execute;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.parallel;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.serial;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.sleepms;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.trajectory;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.SubSystems.V3.Lift;
import org.firstinspires.ftc.teamcode.SubSystems.V3.Turret;
import org.firstinspires.ftc.teamcode.Tasks.SerialTask;

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


    public Pose2d startPoseLeft = new Pose2d(-62.0, 36, 0.0);
    public Trajectory preloadTrajectory, toStackAlign, toStackAlign2, toStack1,
            toHigh1, toStack4, toStackAlign4, toStackAlign3, toHigh2, toHigh3, toHigh4, toStack2, toStack3, toStack5, toStack6, toStackAlign6, toStackAlign5, toHigh5, toHigh6;

    @Override
    public void onInit() {
        drive.setPoseEstimate(startPoseLeft);
        preloadTrajectory = drive.trajectoryBuilder(startPoseLeft)
                .lineToLinearHeading(pose(
                        PRELOAD_X,
                        PRELOAD_Y,
                        PRELOAD_HEADING
                ))
                .build();
        junction = "B2";
        autoAimOffset = -3;

        Turret.fieldSize = 6 * 24;

        toStackAlign = drive.trajectoryBuilder(preloadTrajectory.end())
                .lineToLinearHeading(pose(STACK_ALIGN_X, STACK_ALIGN_Y, STACK_ALIGN_HEADING))
                .build();

        toStack1 = drive.trajectoryBuilder(toStackAlign.end())
                .lineTo(vector(STACK1_X, STACK1_Y - 2.5)).build();

        toHigh1 = drive.trajectoryBuilder(toStack1.end())
                .lineTo(vector(TH1_X + 5.5, TH1_Y)).build();

        toStackAlign2 = drive.trajectoryBuilder(toHigh1.end())
                .lineTo(vector(-12.0, 36.0)).build();

        toStack2 = drive.trajectoryBuilder(toStackAlign2.end())
                .lineTo(vector(STACK1_X, STACK1_Y - 2.5)).build();

        toHigh2 = drive.trajectoryBuilder(toStack2.end())
                .lineTo(vector(TH1_X + 4.8, TH1_Y)).build();

        toStackAlign3 = drive.trajectoryBuilder(toHigh2.end())
                .lineTo(vector(-12.0, 36.0)).build();

        toStack3 = drive.trajectoryBuilder(toStackAlign3.end())
                .lineTo(vector(STACK1_X, STACK1_Y - 2.5)).build();

        toHigh3 = drive.trajectoryBuilder(toStack3.end())
                .lineTo(vector(TH1_X + 5.0, TH1_Y)).build();

        toStackAlign4 = drive.trajectoryBuilder(toHigh3.end())
                .lineTo(vector(-12.0, 36.0)).build();

        toStack4 = drive.trajectoryBuilder(toStackAlign4.end())
                .lineTo(vector(STACK1_X, STACK1_Y - 2.5)).build();

        toHigh4 = drive.trajectoryBuilder(toStack4.end())
                .lineTo(vector(TH1_X + 5.5, TH1_Y)).build();

        toStackAlign5 = drive.trajectoryBuilder(toHigh3.end())
                .lineTo(vector(-12.0, 36.0)).build();

        toStack5 = drive.trajectoryBuilder(toStackAlign4.end())
                .lineTo(vector(STACK1_X, STACK1_Y - 2.5)).build();

        toHigh5 = drive.trajectoryBuilder(toStack4.end())
                .lineTo(vector(TH1_X + 5.5, TH1_Y)).build();

        toStackAlign6 = drive.trajectoryBuilder(toHigh3.end())
                .lineTo(vector(-12.0, 36.0)).build();

        toStack6 = drive.trajectoryBuilder(toStackAlign4.end())
                .lineTo(vector(STACK1_X, STACK1_Y - 2.5)).build();

        toHigh6 = drive.trajectoryBuilder(toStack4.end())
                .lineTo(vector(TH1_X + 5.5, TH1_Y)).build();


        SerialTask drop = serial(
                execute(() -> targetHeight = targetHeight - 500),
                sleepms(150),
                execute(() -> {
                    ct.claw.open();
                    lockOnJunction = false;
                })
//                execute(() -> ct.turret.setTargetAngle(0))
        );


        task = serial(
                //preload
                parallel(
                        trajectory(preloadTrajectory),
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
                        parallel(trajectory(toStackAlign),
                                execute(() -> {
                                    targetHeight = 520;
                                }),
                                serial(
                                        sleepms(450),
                                        execute(() -> {
                                            ct.setCollectPivotAndClawFlip();
                                        })
                                )),
                        //cycle 1
                        execute(() -> ct.turret.setTargetAngle(0)),
                        trajectory(toStack1),
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
                        trajectory(toHigh1),
                        execute(() -> {
                            targetHeight = Lift.MID_POS;
                            lockOnJunction = true;
                            junction = "B2";
                        }),
                        serial(
                                sleepms(500),
                                execute(() -> {
                                    ct.clawFlip.setScore();
                                }))
                ),
                drop,
                serial(
                        parallel(
                                trajectory(toStackAlign2),
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
                        execute(() -> ct.turret.setTargetAngle(0)),
                        trajectory(toStack2),
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
                        trajectory(toHigh2),
                        execute(() -> {
                            autoAimOffset = -2;
                            targetHeight = Lift.MID_POS;
                            lockOnJunction = true;
                            junction = "B2";
                        }),
                        serial(
                                sleepms(500),
                                execute(() -> {
                                    ct.clawFlip.setScore();
                                }))
                ),
                drop,
                serial(
                        parallel(trajectory(toStackAlign3),
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
                        execute(() -> ct.turret.setTargetAngle(0)),
                        trajectory(toStack3),
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
                        trajectory(toHigh3),
                        execute(() -> {
                            targetHeight = Lift.MID_POS;
                            lockOnJunction = true;
                            junction = "B2";
                        }),
                        serial(
                                sleepms(500),
                                execute(() -> {
                                    ct.clawFlip.setScore();
                                }))
                ),
                drop,
                serial(
                        parallel(trajectory(toStackAlign4),
                                execute(() -> {
                                    targetHeight = 230;
                                }),
                                serial(
                                        sleepms(300),
                                        execute(() -> {
                                            ct.setCollectPivotAndClawFlip();
                                        })
                                )),
                        //cycle 1
                        execute(() -> ct.turret.setTargetAngle(0)),
                        trajectory(toStack4),
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
                        trajectory(toHigh4),
                        execute(() -> {
//                            autoAimOffset = -7;
                            targetHeight = Lift.MID_POS;
                            lockOnJunction = true;
                            junction = "B2";
                        }),
                        serial(
                                sleepms(500),
                                execute(() -> {
                                    ct.clawFlip.setScore();
                                }))
                ),
                drop,
                serial(
                        parallel(trajectory(toStackAlign5),
                                execute(() -> {
                                    targetHeight = 100;
                                }),
                                serial(
                                        sleepms(300),
                                        execute(() -> {
                                            ct.setCollectPivotAndClawFlip();
                                        })
                                )),
                        //cycle 1
                        execute(() -> ct.turret.setTargetAngle(0)),
                        trajectory(toStack5),
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
                        trajectory(toHigh5),
                        execute(() -> {
                            targetHeight = Lift.MID_POS;
                            lockOnJunction = true;
                            junction = "B2";
                        }),
                        serial(
                                sleepms(500),
                                execute(() -> {
                                    ct.clawFlip.setScore();
                                }))
                ),
                drop
        );

    }
}

