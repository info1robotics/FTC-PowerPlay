package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.inline;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.pause;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.sync;
import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.trajectory;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Disabled
@Autonomous(name = "Preload MID + Park | Right")
@Config
public class AutoTestRight extends AutoOpMode {
    public Trajectory start_to_align_mid, start_to_mid, go_back, strafe_left, strafe_right, left_front, right_front;
    public static double start_to_mid_x = -35;
    public static double start_to_mid_y = -23.5; // -1 cand e bateria sub 13.3
    public static double start_to_high_heading = 90;

    @Override
    public void onInit() {
        start_to_mid = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(start_to_mid_x, start_to_mid_y, Math.toRadians(start_to_high_heading)))
                .build();

        go_back = drive.trajectoryBuilder(start_to_mid.end(), true)
                .lineToLinearHeading(new Pose2d(-35, -37, Math.toRadians(start_to_high_heading)))
                .build();

        strafe_left = drive.trajectoryBuilder(go_back.end())
                .strafeLeft(15)
                .build();

        left_front = drive.trajectoryBuilder(strafe_left.end())
                .forward(5)
                .build();

        strafe_right = drive.trajectoryBuilder(go_back.end())
                .strafeRight(15)
                .build();

        right_front = drive.trajectoryBuilder(strafe_right.end())
                .forward(5)
                .build();


        task = sync(

                inline(() -> {
                    claw.setState(false);
                }),

                pause(400),

                inline(() -> {
                    linkage.goToLevel(200, 0.27);
                }),

                pause(1000),

                trajectory(start_to_mid),

                inline(() -> {
                    linkage.goToLevel(450, 0.27); // + 30 cand sub 13.3
                }),

                pause(2000),

                inline(() -> {
                    turret.engageBrake();
                    turret.goToAngle(90, 0.75);
                }),

                pause(2000),

                inline(() -> {
                    linkage.goToLevel(300, 0.27); // + 30 cand sub 13.3
                }),

                pause(2000),

                inline(() -> {
                    claw.setState(true);
                }),

                pause(2000),

                inline(() -> {
                    linkage.goToLevel(450, 0.27); // + 30 cand sub 13.3
                }),

                pause(2000),

                inline(() -> {
                    turret.goToAngle(0, 0.75);
                }),

                pause(1000),

                inline(() -> {
                    linkage.goToLevel(0, 0.27); // + 30 cand sub 13.3
                }),

                pause(1000),

                trajectory(go_back),

                inline(() -> {
                    switch(x){
                    case 1: drive.followTrajectory(strafe_left); drive.followTrajectory(left_front); break;
                    case 3: drive.followTrajectory(strafe_right); drive.followTrajectory(right_front); break;
                    }
                })
                );
    }
}
