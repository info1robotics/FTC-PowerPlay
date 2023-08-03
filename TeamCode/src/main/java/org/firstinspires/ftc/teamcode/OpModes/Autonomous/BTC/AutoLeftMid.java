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
            STACK_ALIGN_X = -10.0,
            STACK_ALIGN_Y = 48.0,
            STACK_ALIGN_HEADING = Math.toRadians(-90.0),
            STACK1_X = -10.0,
            STACK1_Y = 58.0,
            TH1_X = -20.5,
            TH1_Y = 32.0,
            TH1_HEADING = Math.toRadians(-30.0);


    public Pose2d startPoseLeft = new Pose2d(-62.0, 36, 0.0);
    public Trajectory preloadTrajectory, toStackAlign, toStackAlign2, toStack1,
            toHigh1, stackCycle1, toHigh2, toHigh3, toHigh4, toStack2, toStack3;
    public TrajectorySequence toHighCycle;

    public SerialTask collect(int height) {
        return serial(
                execute(() -> ct.claw.close()),
                sleepms(20),
                execute(() -> {
                    targetHeight = height;
                }),
                sleepms(50),
                execute(() -> {
                    ct.pivot.setScore();
                })
        );
    }

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

        Turret.fieldSize = 6 * 24;

        toStackAlign = drive.trajectoryBuilder(preloadTrajectory.end())
                .lineToLinearHeading(pose(STACK_ALIGN_X, STACK_ALIGN_Y, STACK_ALIGN_HEADING))
                .build();
        toStack1 = drive.trajectoryBuilder(toStackAlign.end()).lineTo(vector(STACK1_X, STACK1_Y - 2.5)).build();
        toHigh1 = drive.trajectoryBuilder(toStack1.end()).lineTo(vector(TH1_X, TH1_Y)).build();
        stackCycle1 = drive.trajectoryBuilder(toHigh1.end()).splineToConstantHeading
                (vector(STACK1_X, STACK1_Y), -90).build();
        toHigh2 = drive.trajectoryBuilder(stackCycle1.end())
                .lineToConstantHeading(vector(TH1_X, TH1_Y)).build();
        toHighCycle = drive.trajectorySequenceBuilder(stackCycle1.end()).splineToConstantHeading(vector(STACK_ALIGN_X + 3, STACK_ALIGN_Y - 5), -30)
                .lineTo(vector(STACK1_X, STACK1_Y)).build();
        toStackAlign2 = drive.trajectoryBuilder(toHigh1.end()).lineTo(vector(-12.0, 36.0)).build();
        toStack2 = drive.trajectoryBuilder(toStackAlign2.end()).lineTo(vector(STACK1_X+1.5, STACK1_Y+1))
                .build();
        toStack3 = drive.trajectoryBuilder(toStackAlign2.end()).lineTo(vector(STACK1_X + 2, STACK1_Y))
                .build();
        toHigh3 = drive.trajectoryBuilder(toStack2.end()).lineToConstantHeading
                (vector(TH1_X + 6, TH1_Y + 6)).build();
        toHigh4 = drive.trajectoryBuilder(toStack2.end()).lineToConstantHeading
                (vector(TH1_X + 3, TH1_Y - 1)).build();

        SerialTask drop = serial(
                execute(() -> targetHeight = targetHeight - 750),
                sleepms(300),
                execute(() -> {
                    ct.claw.open();
                    lockOnJunction = false;
                }),
                sleepms(350),
                execute(() -> ct.turret.setTargetAngle(0))
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
                sleepms(200),
                serial(
                        parallel(trajectory(toStackAlign),
                                execute(() -> {
                                    targetHeight = 440;
                                }),
                                serial(
                                        sleepms(450),
                                        execute(() -> {
                                            ct.setCollectPivotAndClawFlip();
                                        })
                                )),
                        //cycle 1
                        trajectory(toStack1),
                        execute(() -> ct.claw.close()),
                        sleepms(20),
                        execute(() -> {
                            targetHeight = 800;
                        }),
                        sleepms(50),
                        execute(() -> {
                            ct.pivot.setScore();
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
                sleepms(200),
                serial(
                        parallel(trajectory(toStackAlign2),
                                execute(() -> {
                                    targetHeight = 440;
                                }),
                                serial(
                                        sleepms(300),
                                        execute(() -> {
                                            ct.setCollectPivotAndClawFlip();
                                        })
                                )),
                        //cycle 1
                        trajectory(toStack2),
                        execute(() -> ct.claw.close()),
                        sleepms(20),
                        execute(() -> {
                            targetHeight = 800;
                        }),
                        sleepms(50),
                        execute(() -> {
                            ct.pivot.setScore();
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
                sleepms(200),
                serial(
                        parallel(trajectory(toStackAlign2),
                                execute(() -> {
                                    targetHeight = 400;
                                }),
                                serial(
                                        sleepms(300),
                                        execute(() -> {
                                            ct.setCollectPivotAndClawFlip();
                                        })
                                )),
                        //cycle 1
                        trajectory(toStack2),
                        execute(() -> ct.claw.close()),
                        sleepms(20),
                        execute(() -> {
                            targetHeight = 800;
                        }),
                        sleepms(50),
                        execute(() -> {
                            ct.pivot.setScore();
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
                drop

        );

    }
}

