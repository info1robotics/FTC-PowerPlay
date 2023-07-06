package org.firstinspires.ftc.teamcode.OpModes.Autonomous.CRI;

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
import org.firstinspires.ftc.teamcode.SubSystems.V3.Lift;

@Config
@Autonomous
public class AutoLeft extends AutoBase {
    public static double
            START_TO_HIGH_X_1 = -30,
            START_TO_HIGH_Y_1 = 53.8,
            START_TO_HIGH_HEADING = -90,
            HIGH_TO_STACK_X_1 = -35.6,
            HIGH_TO_STACK_Y_1 = 60.6;
    public Pose2d startPoseLeft = new Pose2d(-90.27, 60);
    public Trajectory highToStack1, startToHigh, highToStack12, stackToHigh1, stackToHigh2, stackToHigh3, stackToHigh4, stackToHigh5, highToStack2, highToStack3, highToStack4, highToStack5;

    @Override
    public void onInit() {
        drive.setPoseEstimate(startPoseLeft);


        startToHigh = drive.trajectoryBuilder(startPoseLeft)
                .lineToLinearHeading(pose(
                        START_TO_HIGH_X_1,
                        START_TO_HIGH_Y_1,
                        Math.toRadians(START_TO_HIGH_HEADING)
                ))
                .build();

        highToStack1 = drive.trajectoryBuilder(startToHigh.end(), true)
                .lineToConstantHeading(vector(
                        HIGH_TO_STACK_X_1,
                        HIGH_TO_STACK_Y_1
                ))
                .build();

        highToStack12 = drive.trajectoryBuilder(highToStack1.end(), true)
                .lineToConstantHeading(vector(
                        -40.5,
                        79
                ))
                .build();

        stackToHigh1 = drive.trajectoryBuilder(highToStack12.end())
                .lineToConstantHeading(vector(
                        -35.85,
                        46.57
                ))
                .build();

        highToStack2 = drive.trajectoryBuilder(stackToHigh1.end(), true)
                .lineToConstantHeading(vector(-40.5, 79))
                .build();

        stackToHigh2 = drive.trajectoryBuilder(highToStack2.end())
                .lineToConstantHeading(vector(
                        -35.0,
                        46
                ))
                .build();

        highToStack3 = drive.trajectoryBuilder(stackToHigh2.end(), true)
                .lineToConstantHeading(vector(-40.5, 79))
                .build();

        stackToHigh3 = drive.trajectoryBuilder(highToStack3.end())
                .lineToConstantHeading(vector(
                        -35.5,
                        46.5
                ))
                .build();

        highToStack4 = drive.trajectoryBuilder(stackToHigh3.end(), true)
                .lineToConstantHeading(vector(-40.5, 79))
                .build();

        stackToHigh4 = drive.trajectoryBuilder(highToStack4.end())
                .lineToConstantHeading(vector(
                        -35.5,
                        46.5
                ))
                .build();

        highToStack5 = drive.trajectoryBuilder(stackToHigh4.end(), true)
                .lineToConstantHeading(vector(-40.5, 79))
                .build();

        stackToHigh5 = drive.trajectoryBuilder(highToStack5.end())
                .lineToConstantHeading(vector(
                        -35.5,
                        46.5
                ))
                .build();


        ct.claw.close();

        task = serial(
                parallel(
                        trajectory(startToHigh),
                        serial(
                                execute(() -> {
                                    junction = "B3";
                                    lockOnJunction = false;
                                    ct.turret.setTargetAngle(30);
                                }),
                                sleepms(200),
                                execute(() -> {
                                    targetHeight = Lift.HIGH_POS;
                                })
                        )
                ),
                sleepms(200),
                execute(() -> {
                    targetHeight = Lift.HIGH_POS - 600;
                }),
                sleepms(100),
                execute(() -> {
                    ct.claw.open();
                }),
                sleepms(100),
                parallel(
                        execute(() -> {
                            lockOnJunction = false;
                            ct.clawFlip.setCollect();
                            ct.pivot.setCollect();
                            targetHeight = 500;
                            ct.turret.setTargetAngle(0);
                        }),
                        trajectory(highToStack1)
                ),
//                sleepms(200),
                trajectory(highToStack12),
                execute(() -> {
                    ct.claw.close();
                }),
                sleepms(225),
                execute(() -> {
                    targetHeight = 1000;
                }),
                sleepms(100),
                execute(() -> {
                    ct.pivot.setScore();
                    ct.turret.setTurretVelocity(.64);
                    junction = "B3";
                    lockOnJunction = true;
                }),
                parallel(
                        trajectory(stackToHigh1),
                        serial(
                                sleepms(500),
                                execute(() -> {
                                    ct.clawFlip.setScore();
                                }),
                                sleepms(650),
                                execute(() -> {
                                    targetHeight = Lift.HIGH_POS;
                                })
                        )
                ),
                sleepms(240),
                execute(() -> {
                    targetHeight = Lift.HIGH_POS - 600;
                }),
                sleepms(360),
                execute(() -> {
                    ct.claw.open();
                }),

                // cycle 2
                parallel(
                        execute(() -> {
                            lockOnJunction = false;
                            ct.clawFlip.setCollect();
                            ct.pivot.setCollect();
                            targetHeight = 400;
                            ct.turret.setTargetAngle(0);
                        })
                ),
                sleepms(200),
                trajectory(highToStack2),
                execute(() -> {
                    ct.claw.close();
                }),
                sleepms(225),
                execute(() -> {
                    targetHeight = 1000;
                }),
                sleepms(100),
                execute(() -> {
                    ct.pivot.setScore();
                    ct.turret.setTurretVelocity(.64);
                    junction = "B3";
                    lockOnJunction = true;
                }),
                parallel(
                        trajectory(stackToHigh2),
                        serial(
                                sleepms(500),
                                execute(() -> {
                                    ct.clawFlip.setScore();
                                }),
                                sleepms(650),
                                execute(() -> {
                                    targetHeight = Lift.HIGH_POS;
                                })
                        )
                ),
                sleepms(240),
                execute(() -> {
                    targetHeight = Lift.HIGH_POS - 600;
                }),
                sleepms(360),
                execute(() -> {
                    ct.claw.open();
                }),
                // cycle 3
                parallel(
                        execute(() -> {
                            lockOnJunction = false;
                            ct.clawFlip.setCollect();
                            ct.pivot.setCollect();
                            targetHeight = 300;
                            ct.turret.setTargetAngle(0);
                        })
                ),
                sleepms(200),
                trajectory(highToStack3),
                execute(() -> {
                    ct.claw.close();
                }),
                sleepms(225),
                execute(() -> {
                    targetHeight = 1000;
                }),
                sleepms(100),
                execute(() -> {
                    ct.pivot.setScore();
                    ct.turret.setTurretVelocity(.64);
                    junction = "B3";
                    lockOnJunction = true;
                }),
                parallel(
                        trajectory(stackToHigh3),
                        serial(
                                sleepms(500),
                                execute(() -> {
                                    ct.clawFlip.setScore();
                                }),
                                sleepms(650),
                                execute(() -> {
                                    targetHeight = Lift.HIGH_POS;
                                })
                        )
                ),
                sleepms(240),
                execute(() -> {
                    targetHeight = Lift.HIGH_POS - 600;
                }),
                sleepms(360),
                execute(() -> {
                    ct.claw.open();
                }),


                // cycle 4
                parallel(
                        execute(() -> {
                            lockOnJunction = false;
                            ct.clawFlip.setCollect();
                            ct.pivot.setCollect();
                            targetHeight = 200;
                            ct.turret.setTargetAngle(0);
                        })
                ),
                sleepms(200),
                trajectory(highToStack4),
                execute(() -> {
                    ct.claw.close();
                }),
                sleepms(225),
                execute(() -> {
                    targetHeight = 1000;
                }),
                sleepms(100),
                execute(() -> {
                    ct.pivot.setScore();
                    ct.turret.setTurretVelocity(.64);
                    junction = "B3";
                    lockOnJunction = true;
                }),
                parallel(
                        trajectory(stackToHigh4),
                        serial(
                                sleepms(500),
                                execute(() -> {
                                    ct.clawFlip.setScore();
                                }),
                                sleepms(650),
                                execute(() -> {
                                    targetHeight = Lift.HIGH_POS;
                                })
                        )
                ),
                sleepms(240),
                execute(() -> {
                    targetHeight = Lift.HIGH_POS - 600;
                }),
                sleepms(360),
                execute(() -> {
                    ct.claw.open();
                }),

                // cycle 5
                parallel(
                        execute(() -> {
                            lockOnJunction = false;
                            ct.clawFlip.setCollect();
                            ct.pivot.setCollect();
                            targetHeight = 200;
                            ct.turret.setTargetAngle(0);
                        })
                ),
                sleepms(200),
                trajectory(highToStack5),
                execute(() -> {
                    ct.claw.close();
                }),
                sleepms(225),
                execute(() -> {
                    targetHeight = 1000;
                }),
                sleepms(100),
                execute(() -> {
                    ct.pivot.setScore();
                    ct.turret.setTurretVelocity(.64);
                    junction = "B3";
                    lockOnJunction = true;
                }),
                parallel(
                        trajectory(stackToHigh5),
                        serial(
                                sleepms(500),
                                execute(() -> {
                                    ct.clawFlip.setScore();
                                }),
                                sleepms(650),
                                execute(() -> {
                                    targetHeight = Lift.HIGH_POS;
                                })
                        )
                ),
                sleepms(240),
                execute(() -> {
                    targetHeight = Lift.HIGH_POS - 600;
                }),
                sleepms(360),
                execute(() -> {
                    ct.claw.open();
                })
        );
    }
}
