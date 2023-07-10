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
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.SubSystems.V3.Lift;

import java.util.Arrays;
import java.util.Collections;

@Config
@Autonomous
public class AutoRight extends AutoBase {
    public static double
            START_TO_HIGH_X_1 = -26,
            START_TO_HIGH_Y_1 = -55,
            START_TO_HIGH_HEADING = 90,
            HIGH_TO_STACK_X_1 = -35.6,
            HIGH_TO_STACK_Y_1 = -60.6;
    public Pose2d startPoseLeft = new Pose2d(-85.27, -59.5);
    public Trajectory highToStack1, highToStack12, stackToHigh1, stackToHigh2, stackToHigh3, stackToHigh4, stackToHigh5, highToStack2, highToStack3, highToStack4, highToStack5;
    public TrajectorySequence startToHigh, parkTest;
    TrajectoryVelocityConstraint fastConstraint = new MinVelocityConstraint(Collections.singletonList(
            new TranslationalVelocityConstraint(120)
    ));
    ProfileAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(120);

    @Override
    public void onInit() {
        drive.setPoseEstimate(startPoseLeft);
        autoAimOffset = -5;


        startToHigh = drive.trajectorySequenceBuilder(startPoseLeft)
                .lineToLinearHeading(pose(
                        START_TO_HIGH_X_1,
                        START_TO_HIGH_Y_1,
                        Math.toRadians(START_TO_HIGH_HEADING)
                ))
                .build();

        highToStack1 = drive.trajectoryBuilder(startToHigh.end(), true)
                .lineToConstantHeading(vector(
                        -38.3,
                        -80
                ))
                .addSpatialMarker(vector(-38.3, -79.5), () -> {
                    ct.claw.close();
                })
                .build();

        stackToHigh1 = drive.trajectoryBuilder(highToStack1.end())
                .lineToConstantHeading(vector(
                        -31,
                        -51.5
                ))
                .addSpatialMarker(vector(
                        -31,
                        -51.5
                ), () -> {
                    // turret not working targetHeight = Lift.HIGH_POS - 600;
                })
                .build();

        highToStack2 = drive.trajectoryBuilder(stackToHigh1.end(), true)
                .lineToConstantHeading(vector(
                        -37,
                        -79
                ))
                .addSpatialMarker(vector(-38.3, -79.5), () -> {
                    ct.claw.close();
                })
                .build();

        stackToHigh2 = drive.trajectoryBuilder(highToStack2.end())
                .lineToConstantHeading(vector(
                        -31,
                        -51.5
                ))
                .addSpatialMarker(vector(
                        -31,
                        -51.5
                ), () -> {
                    // turret not working targetHeight = Lift.HIGH_POS - 600;
                })
                .build();

        highToStack3 = drive.trajectoryBuilder(stackToHigh2.end(), true)
                .lineToConstantHeading(vector(
                        -37,
                        -79
                ))
                .addSpatialMarker(vector(-38.3, -79.5), () -> {
                    ct.claw.close();
                })
                .build();

        stackToHigh3 = drive.trajectoryBuilder(highToStack3.end())
                .lineToConstantHeading(vector(
                        -31,
                        -51.5
                ))
                .addSpatialMarker(vector(
                        -31,
                        -51.5
                ), () -> {
                    // turret not working targetHeight = Lift.HIGH_POS - 600;
                })
                .build();

        highToStack4 = drive.trajectoryBuilder(stackToHigh3.end(), true)
                .lineToConstantHeading(vector(
                        -37,
                        -79
                ))
                .addSpatialMarker(vector(-38.3, 79.5), () -> {
                    ct.claw.close();
                })
                .build();

        stackToHigh4 = drive.trajectoryBuilder(highToStack4.end())
                .lineToConstantHeading(vector(
                        -31,
                        -51.5
                ))
                .addSpatialMarker(vector(
                        -31,
                        -51.5
                ), () -> {
                    // turret not working targetHeight = Lift.HIGH_POS - 600;
                })
                .build();

        highToStack5 = drive.trajectoryBuilder(stackToHigh4.end(), true)
                .lineToConstantHeading(vector(
                        -37,
                        -79
                ))
                .addSpatialMarker(vector(-38.3, -79.5), () -> {
                    ct.claw.close();
                })
                .build();

        stackToHigh5 = drive.trajectoryBuilder(highToStack5.end())
                .lineToConstantHeading(vector(
                        -31,
                        -51.5
                ))
                .addSpatialMarker(vector(
                        -31,
                        -51.5
                ), () -> {
                    // turret not working targetHeight = Lift.HIGH_POS - 600;
                })
                .build();

        parkTest = drive.trajectorySequenceBuilder(stackToHigh5.end())
                .lineToConstantHeading(vector(-39.4, -88.4))
                .setVelConstraint(fastConstraint)
                .setAccelConstraint(accelConstraint)
                .build();


        ct.claw.close();

        task = serial(
                parallel(
                        trajectorySequence(startToHigh),
                        serial(
                                execute(() -> {
                                    junction = "F3";
                                    lockOnJunction = false;
                                    ct.turret.setTargetAngle(-30);
                                }),
                                sleepms(201),
                                execute(() -> {
                                    // turret not working targetHeight = Lift.HIGH_POS;
                                })
                        )
                ),
                sleepms(70),
                execute(() -> {
                    // turret not working targetHeight = Lift.HIGH_POS - 600;
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
                            // turret not working targetHeight = 500;
                            ct.turret.setTargetAngle(6.5);
                        })
                ),
                sleepms(20),
                trajectory(highToStack1),
                execute(() -> {
                    // turret not working targetHeight = 1000;
                }),
                sleepms(20),
                execute(() -> {
                    ct.pivot.setScore();
                    ct.turret.setTurretVelocity(.24);
                    junction = "F3";
                    lockOnJunction = true;
                }),
                parallel(
                        trajectory(stackToHigh1),
                        serial(
                                sleepms(500),
                                execute(() -> {
                                    ct.clawFlip.setScore();
                                }),
                                sleepms(350),
                                execute(() -> {
                                    // turret not working targetHeight = Lift.HIGH_POS;
                                })
                        )
                ),

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
                            // turret not working targetHeight = 400;
                            ct.turret.setTargetAngle(0);
                        })
                ),
                sleepms(20),
                trajectory(highToStack2),
                execute(() -> {
                    // turret not working targetHeight = 1000;
                }),
                sleepms(20),
                execute(() -> {
                    ct.pivot.setScore();
                    ct.turret.setTurretVelocity(.24);
                    junction = "F3";
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
                                    // turret not working targetHeight = Lift.HIGH_POS;
                                })
                        )
                ),

                execute(() -> {
                    ct.claw.open();
                }),
                // cycle 3
                parallel(
                        execute(() -> {
                            lockOnJunction = false;
                            ct.clawFlip.setCollect();
                            ct.pivot.setCollect();
                            // turret not working targetHeight = 300;
                            ct.turret.setTurretVelocity(.44);
                            ct.turret.setTargetAngle(0);
                        })
                ),
                sleepms(20),
                trajectory(highToStack3),
                execute(() -> {
                    // turret not working targetHeight = 1000;
                }),
                sleepms(20),
                execute(() -> {
                    ct.pivot.setScore();
                    ct.turret.setTurretVelocity(.24);
                    junction = "F3";
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
                                    // turret not working targetHeight = Lift.HIGH_POS;
                                })
                        )
                ),

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
                            // turret not working targetHeight = 200;
                            ct.turret.setTargetAngle(0);
                        })
                ),
                sleepms(20),
                trajectory(highToStack4),
                execute(() -> {
                    // turret not working targetHeight = 1000;
                }),
                sleepms(20),
                execute(() -> {
                    ct.pivot.setScore();
                    ct.turret.setTurretVelocity(.24);
                    junction = "F3";
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
                                    // turret not working targetHeight = Lift.HIGH_POS;
                                })
                        )
                ),
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
                            // turret not working targetHeight = 200;
                            ct.turret.setTargetAngle(0);
                        })
                ),
                sleepms(20),
                trajectory(highToStack5),
                execute(() -> {
                    // turret not working targetHeight = 1000;
                }),
                sleepms(20),
                execute(() -> {
                    ct.pivot.setScore();
                    ct.turret.setTurretVelocity(.24);
                    junction = "F3";
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
                                    // turret not working targetHeight = Lift.HIGH_POS;
                                })
                        )
                ),

                execute(() -> {
                    ct.claw.open();
                }),
                parallel(
                        execute(() -> {
                            lockOnJunction = false;
                            ct.turret.setTurretVelocity(0);
                            ct.turret.setTargetAngle(0);
                            ct.clawFlip.setScore();
                            // turret not working targetHeight = 0;
                        })
//                        trajectorySequence(parkTest)

                )
        );
    }
}
