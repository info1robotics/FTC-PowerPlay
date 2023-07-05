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
            START_TO_HIGH_X_1 = -28,
            START_TO_HIGH_Y_1 = 54,
            START_TO_HIGH_HEADING = -90,
            HIGH_TO_STACK_X_1 = -40,
            HIGH_TO_STACK_Y_1 = 60;
    public Pose2d startPoseLeft = new Pose2d(-90.27, 60);
    public Trajectory highToStack1, startToHigh, highToStack12, stackToHigh1;

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
                        -39,
                        79
                ))
                .build();

        stackToHigh1 = drive.trajectoryBuilder(highToStack12.end())
                .lineToConstantHeading(vector(
                        -32,
                        55
                ))
                .build();


        ct.claw.close();

        task = serial(
                parallel(
                        trajectory(startToHigh),
                        serial(
                                execute(() -> {
                                    ct.turret.setTargetAngle(20);
                                    ct.turret.setPower(1.0);
                                }),
                                sleepms(200),
                                execute(() -> {
                                    targetHeight = Lift.HIGH_POS;
                                })
                        )
                ),
                sleepms(50),
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
                            ct.clawFlip.setCollect();
                            ct.pivot.setCollect();
                            targetHeight = 500;
                            ct.turret.setTargetAngle(0);
                        }),
                        trajectory(highToStack1)
                ),
                trajectory(highToStack12),
                execute(() -> {
                    ct.claw.close();
                }),
                sleepms(100),
                execute(() -> {
                    targetHeight = 1000;
                }),
                sleepms(100),
                execute(() -> {
                    ct.pivot.setScore();
                    ct.turret.setTargetAngle(30);

                }),
                parallel(
                        trajectory(stackToHigh1),
                        serial(
                        sleepms(500),
                        execute(() -> {
                            ct.clawFlip.setScore();
                            targetHeight = Lift.HIGH_POS;
                        }))
                ),
                sleepms(100),
                execute(() -> {
                    targetHeight = Lift.HIGH_POS - 600;
                }),
                sleepms(100),
                execute(() -> {
                    ct.claw.open();
                })

//                execute(() -> {
//                    targetHeight = Lift.MID_POS;
//                    ct.claw.open();
//                }),
//                sleepms(50),
//                parallel(
//                        trajectory(highToStack1),
//                        execute(() -> {
//                            targetHeight = 50;
//                            lockOnJunction = false;
//                            ct.setCollectPivotAndClawFlip();
//                        })
//                )
        );
    }
}
