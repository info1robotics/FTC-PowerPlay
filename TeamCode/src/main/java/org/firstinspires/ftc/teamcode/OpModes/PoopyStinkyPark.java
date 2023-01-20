package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.EOCV.f41h12.AprilTagDetection_41h12;
import org.firstinspires.ftc.teamcode.Kinematics.EncoderAuto;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

@Autonomous(name="Park Only | Both Sides")
public class PoopyStinkyPark extends LinearOpMode {
    Turret turret;
    AprilTagDetection_41h12 atag;
    int x = 2;
    @Override
    public void runOpMode() throws InterruptedException {
        atag = new AprilTagDetection_41h12(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        turret = new Turret(this);

        Trajectory align = drive.trajectoryBuilder(new Pose2d())
                .forward(27)
                .build();
        TrajectorySequence left = drive.trajectorySequenceBuilder(align.end())
                .strafeLeft(15)
                .forward(5)
                .build();
        TrajectorySequence right = drive.trajectorySequenceBuilder(align.end())
                .strafeRight(15)
                .forward(5)
                .build();

        while (!isStarted() && !isStopRequested()) {
            turret.engageBrake();
            turret.goToAngle(0, 1.0);
        atag.detectZone();
        x = atag.getZone();
        }
        drive.followTrajectory(align);
        switch(x){
            case 1: drive.followTrajectorySequence(left); break;
            case 2: break;
            case 3: drive.followTrajectorySequence(right); break;
        }
        drive.turn(180f);
    }
}
