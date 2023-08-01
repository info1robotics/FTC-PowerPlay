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
import org.firstinspires.ftc.teamcode.SubSystems.V3.Pivot;

import java.util.Arrays;
import java.util.Collections;

@Config
@Autonomous
public class AutoPark extends AutoBase {
    public static double
            START_TO_HIGH_X_1 = -26,
            START_TO_HIGH_Y_1 = 55,
            START_TO_HIGH_HEADING = -90,
            HIGH_TO_STACK_X_1 = -35.6,
            HIGH_TO_STACK_Y_1 = 60.6;
    public Pose2d startPoseLeft = new Pose2d(-88.27, 59.5);
    public Trajectory highToStack1, highToStack12, stackToHigh1, stackToHigh2, stackToHigh3, stackToHigh4, stackToHigh5, highToStack2, highToStack3, highToStack4, highToStack5;
    public TrajectorySequence startToHigh, parkLeft, parkRight, parkMid;
    TrajectoryVelocityConstraint fastConstraint = new MinVelocityConstraint(Collections.singletonList(
            new TranslationalVelocityConstraint(120)
    ));
    ProfileAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(120);
    ProfileAccelerationConstraint accelConstraintPreload = new ProfileAccelerationConstraint(45);

    @Override
    public void onInit() {
        drive.setPoseEstimate(startPoseLeft);

        startToHigh = drive.trajectorySequenceBuilder(startPoseLeft)
                .lineToConstantHeading(vector(
                        -35,
                        55
                ))
                .build();

        highToStack1 = drive.trajectoryBuilder(startToHigh.end(), true)
                .lineToConstantHeading(vector(
                        -41.5,
                        81.5
                ))
                .addSpatialMarker(vector(-41.5, 81.5), () -> {
                    ct.claw.close();
                })
                .build();

        stackToHigh1 = drive.trajectoryBuilder(highToStack1.end())
                .lineToConstantHeading(vector(
                        -30,
                        50.8
                ))
                .addSpatialMarker(vector(
                        -30,
                        50.8
                ), () -> {
                    targetHeight = Lift.HIGH_POS - 100 - 600;
                })
                .build();

        highToStack2 = drive.trajectoryBuilder(stackToHigh1.end(), true)
                .lineToConstantHeading(vector(
                        -36.5,
                        81.5
                ))
                .addSpatialMarker(vector(-36.5, 81.5), () -> {
                    ct.claw.close();
                })
                .build();

        stackToHigh2 = drive.trajectoryBuilder(highToStack2.end())
                .lineToConstantHeading(vector(
                        -30,
                        50.8
                ))
                .addSpatialMarker(vector(
                        -30,
                        50.8
                ), () -> {
                    targetHeight = Lift.HIGH_POS - 100 - 600;
                })
                .build();

        highToStack3 = drive.trajectoryBuilder(stackToHigh2.end(), true)
                .lineToConstantHeading(vector(
                        -36.3,
                        81.6
                ))
                .addSpatialMarker(vector(-36.3, 81.6), () -> {
                    ct.claw.close();
                })
                .build();

        stackToHigh3 = drive.trajectoryBuilder(highToStack3.end())
                .lineToConstantHeading(vector(
                        -30,
                        50.8
                ))
                .addSpatialMarker(vector(
                        -30,
                        50.8
                ), () -> {
                    targetHeight = Lift.HIGH_POS - 100 - 600;
                })
                .build();

        highToStack4 = drive.trajectoryBuilder(stackToHigh3.end(), true)
                .lineToConstantHeading(vector(
                        -36.3,
                        81.6
                ))
                .addSpatialMarker(vector(-36.3, 81.6), () -> {
                    ct.claw.close();
                })
                .build();

        stackToHigh4 = drive.trajectoryBuilder(highToStack4.end())
                .lineToConstantHeading(vector(
                        -30,
                        50.8
                ))
                .addSpatialMarker(vector(
                        -30,
                        50.8
                ), () -> {
                    targetHeight = Lift.HIGH_POS - 100 - 600;
                })
                .build();

        highToStack5 = drive.trajectoryBuilder(stackToHigh4.end(), true)
                .lineToConstantHeading(vector(
                        -36.3,
                        81.6
                ))
                .addSpatialMarker(vector(-36.3, 81.5), () -> {
                    ct.claw.close();
                })
                .build();

        stackToHigh5 = drive.trajectoryBuilder(highToStack5.end())
                .lineToConstantHeading(vector(
                        -30,
                        50.8
                ))
                .addSpatialMarker(vector(
                        -30,
                        50.8
                ), () -> {
                    targetHeight = Lift.HIGH_POS - 100 - 600;
                })
                .build();

        parkLeft = drive.trajectorySequenceBuilder(stackToHigh5.end())
                .lineToConstantHeading(vector(-41.4, 65 + 25))
                .setVelConstraint(fastConstraint)
                .setAccelConstraint(accelConstraint)
                .build();

        parkRight = drive.trajectorySequenceBuilder(stackToHigh5.end())
                .lineToConstantHeading(vector(-41.4, 65 - 25))
                .setVelConstraint(fastConstraint)
                .setAccelConstraint(accelConstraint)
                .build();

        parkMid = drive.trajectorySequenceBuilder(stackToHigh5.end())
                .lineToConstantHeading(vector(-41.4, 65))
                .setVelConstraint(fastConstraint)
                .setAccelConstraint(accelConstraint)
                .build();


        ct.claw.close();

        task = serial(
                execute(() -> drive.followTrajectorySequence(startToHigh)),
                execute(() -> {
                    if (preferredZone == 1) {
                        drive.followTrajectorySequence(parkLeft);
                    } else if (preferredZone == 2) {
                        drive.followTrajectorySequence(parkMid);
                    } else if (preferredZone == 3) {
                        drive.followTrajectorySequence(parkRight);
                    } else {
                        drive.followTrajectorySequence(parkRight);
                    }
                })
        );
    }
}
