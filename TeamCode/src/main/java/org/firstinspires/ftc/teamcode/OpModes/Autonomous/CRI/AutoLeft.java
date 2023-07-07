package org.firstinspires.ftc.teamcode.OpModes.Autonomous.CRI;

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
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.SubSystems.V3.Lift;

import java.util.Arrays;

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
    public Trajectory highToStack1, highToStack12, stackToHigh1, stackToHigh2, stackToHigh3, stackToHigh4, stackToHigh5, highToStack2, highToStack3, highToStack4, highToStack5;
    public TrajectorySequence startToHigh, parkTest;
    TrajectoryVelocityConstraint fastConstraint = new MinVelocityConstraint(Arrays.asList(
            new TranslationalVelocityConstraint(120)
    ));
    ProfileAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(120);
    @Override
    public void onInit() {
        drive.setPoseEstimate(startPoseLeft);


        startToHigh = drive.trajectorySequenceBuilder(startPoseLeft)
                .lineToLinearHeading(pose(
                        START_TO_HIGH_X_1,
                        START_TO_HIGH_Y_1,
                        Math.toRadians(START_TO_HIGH_HEADING)
                ))
                .build();

        highToStack1 = drive.trajectoryBuilder(startToHigh.end(), true)
                .lineToConstantHeading(vector(
                        -42.3,
                        79
                ))
                .addSpatialMarker(vector(-39.3, 78), () -> {
                    ct.claw.close();
                })
                .build();

        stackToHigh1 = drive.trajectoryBuilder(highToStack1.end())
                .lineToConstantHeading(vector(
                        -35.85,
                        46.57
                ))
                .build();

        highToStack2 = drive.trajectoryBuilder(stackToHigh1.end(), true)
                .lineToConstantHeading(vector(-39.3, 79))
                .addSpatialMarker(vector(-39.3, 78), () -> {
                    ct.claw.close();
                })
                .build();

        stackToHigh2 = drive.trajectoryBuilder(highToStack2.end())
                .lineToConstantHeading(vector(
                        -34.0,
                        46
                ))
                .build();

        highToStack3 = drive.trajectoryBuilder(stackToHigh2.end(), true)
                .lineToConstantHeading(vector(-39.3, 79))
                .addSpatialMarker(vector(-39.3, 78), () -> {
                    ct.claw.close();
                })
                .build();

        stackToHigh3 = drive.trajectoryBuilder(highToStack3.end())
                .lineToConstantHeading(vector(
                        -34.8,
                        46.5
                ))
                .build();

        highToStack4 = drive.trajectoryBuilder(stackToHigh3.end(), true)
                .lineToConstantHeading(vector(-39.4, 79))
                .addSpatialMarker(vector(-39.3, 78), () -> {
                    ct.claw.close();
                })
                .build();

        stackToHigh4 = drive.trajectoryBuilder(highToStack4.end())
                .lineToConstantHeading(vector(
                        -34.8,
                        46.5
                ))
                .build();

        highToStack5 = drive.trajectoryBuilder(stackToHigh4.end(), true)
                .lineToConstantHeading(vector(-39.4, 79.4))
                .addSpatialMarker(vector(-39.3, 78), () -> {
                    ct.claw.close();
                })
                .build();

        stackToHigh5 = drive.trajectoryBuilder(highToStack5.end())
                .lineToConstantHeading(vector(
                        -34.8,
                        46.5
                ))
                .build();

        parkTest = drive.trajectorySequenceBuilder(stackToHigh5.end())
                .lineToConstantHeading(vector(-39.4, 79.4))
                .setVelConstraint(fastConstraint)
                .setAccelConstraint(accelConstraint)
                .build();


        ct.claw.close();

        task = serial(
                parallel(
                        trajectorySequence(startToHigh),
                        serial(
                                execute(() -> {
                                    junction = "B3";
                                    lockOnJunction = false;
                                    ct.turret.setTargetAngle(30);
                                }),
                                sleepms(201),
                                execute(() -> {
                                    targetHeight = Lift.HIGH_POS;
                                })
                        )
                ),
                sleepms(70),
                execute(() -> {
                    targetHeight = Lift.HIGH_POS - 600;
                }),
                sleepms(50),
                execute(() -> {
                    ct.claw.open();
                }),
                sleepms(20),
                parallel(
                        execute(() -> {
                            lockOnJunction = false;
                            ct.clawFlip.setCollect();
                            ct.pivot.setCollect();
                            ct.turret.setTurretVelocity(.44);
                            targetHeight = 500;
                            ct.turret.setTargetAngle(0);
                        })
                ),
                sleepms(20),
                trajectory(highToStack1),
                execute(() -> {
                    targetHeight = 1000;
                }),
                sleepms(20),
                execute(() -> {
                    ct.pivot.setScore();
                    ct.turret.setTurretVelocity(.24);
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
                sleepms(10),
                execute(() -> {
                    targetHeight = Lift.HIGH_POS - 600;
                }),
                sleepms(20),
                execute(() -> {
                    ct.claw.open();
                }),

                // cycle 2
                parallel(
                        execute(() -> {
                            lockOnJunction = false;
                            ct.clawFlip.setCollect();
                            ct.pivot.setCollect();
                            ct.turret.setTurretVelocity(.44);
                            targetHeight = 400;
                            ct.turret.setTargetAngle(0);
                        })
                ),
                sleepms(20),
                trajectory(highToStack2),
                execute(() -> {
                    targetHeight = 1000;
                }),
                sleepms(20),
                execute(() -> {
                    ct.pivot.setScore();
                    ct.turret.setTurretVelocity(.24);
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
                sleepms(10),
                execute(() -> {
                    targetHeight = Lift.HIGH_POS - 600;
                }),
                sleepms(20),
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
                            ct.turret.setTurretVelocity(.44);
                            ct.turret.setTargetAngle(0);
                        })
                ),
                sleepms(20),
                trajectory(highToStack3),
                execute(() -> {
                    targetHeight = 1000;
                }),
                sleepms(20),
                execute(() -> {
                    ct.pivot.setScore();
                    ct.turret.setTurretVelocity(.24);
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
                sleepms(10),
                execute(() -> {
                    targetHeight = Lift.HIGH_POS - 600;
                }),
                sleepms(20),
                execute(() -> {
                    ct.claw.open();
                }),


                // cycle 4
                parallel(
                        execute(() -> {
                            lockOnJunction = false;
                            ct.clawFlip.setCollect();
                            ct.pivot.setCollect();
                            ct.turret.setTurretVelocity(.44);
                            targetHeight = 200;
                            ct.turret.setTargetAngle(0);
                        })
                ),
                sleepms(20),
                trajectory(highToStack4),
                execute(() -> {
                    targetHeight = 1000;
                }),
                sleepms(20),
                execute(() -> {
                    ct.pivot.setScore();
                    ct.turret.setTurretVelocity(.24);
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
                sleepms(10),
                execute(() -> {
                    targetHeight = Lift.HIGH_POS - 600;
                }),
                sleepms(20),
                execute(() -> {
                    ct.claw.open();
                }),

                // cycle 5
                parallel(
                        execute(() -> {
                            lockOnJunction = false;
                            ct.clawFlip.setCollect();
                            ct.pivot.setCollect();
                            ct.turret.setTurretVelocity(.44);
                            targetHeight = 200;
                            ct.turret.setTargetAngle(0);
                        })
                ),
                sleepms(20),
                trajectory(highToStack5),
                execute(() -> {
                    targetHeight = 1000;
                }),
                sleepms(20),
                execute(() -> {
                    ct.pivot.setScore();
                    ct.turret.setTurretVelocity(.24);
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
                sleepms(10),
                execute(() -> {
                    targetHeight = Lift.HIGH_POS - 600;
                }),
                sleepms(20),
                execute(() -> {
                    ct.claw.open();
                }),
                parallel(
                        trajectorySequence(parkTest),
                        serial(
                                execute(() -> {
                                    ct.turret.setTurretVelocity(0);
                                    ct.turret.setTargetAngle(0);
                                    ct.clawFlip.setScore();
                                }),
                                execute(() -> {
                                    targetHeight = 0;
                                })
                        )
                )
        );
    }
}
