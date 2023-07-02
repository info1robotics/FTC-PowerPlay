package org.firstinspires.ftc.teamcode.OpModes.Autonomous.CRI;

import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.execute;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.parallel;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.serial;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.sleepms;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.trajectory;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.SubSystems.V3.Lift;

@Autonomous
public class AutoLeft extends AutoBase {
    public static double
            START_TO_HIGH_X = -23,
            START_TO_HIGH_Y = 55;
    public Pose2d startPoseLeft = new Pose2d(-90.27, 60);
    public Trajectory startToMid;

    @Override
    public void onInit() {
        drive.setPoseEstimate(startPoseLeft);

        startToMid = drive.trajectoryBuilder(startPoseLeft)
                .lineToConstantHeading(new Vector2d(START_TO_HIGH_X, START_TO_HIGH_Y))
                .build();

        ct.claw.close();

        task = serial(
                parallel(
                        trajectory(startToMid),
                        serial(
                                sleepms(200),
                                execute(() -> {
                                    targetHeight = Lift.HIGH_POS;
                                    junction = "B3";
                                    lockOnJunction = true;
                                })
                        )
                ),
                execute(() -> {
                    targetHeight = Lift.MID_POS;
                    ct.claw.open();
                }),
                sleepms(50),
                execute(() -> {
                    targetHeight = Lift.LOW_POS;
                    lockOnJunction = false;
                })
        );
    }
}
