package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.Tasks.TaskBuilder.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Auto Merisor")
@Config
public class AutoTest extends AutoOpMode {
    public Trajectory myTrajectory;
    public static double x = -34;
    public static double y = 3.5;
    @Override
    public void onInit() {
        myTrajectory = drive.trajectoryBuilder(startPose)
//                .lineToConstantHeading(new Vector2d(x, y))
                .lineToLinearHeading(new Pose2d(-33.5, 2.5, Math.toRadians(50)))
                .build();

        task = async (
                trajectory(myTrajectory),
                sync(
                    pause(500),
                    inline(() -> {
                        linkage.goToLevel(550, 0.8);
                        pause(100);
                        turret.goToAngle(45, 1.0);
                        pause(500);
                        turret.goToAngle(0, 1.0);
                    })
                )
        );
    }
}
