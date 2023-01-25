package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.EOCV.f41h12.AprilTagDetection_41h12;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.Linkage;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

@Disabled
@Autonomous(name="Auto Anti SH | Both Sides")
public class AutoAntiSH extends LinearOpMode {
    Turret turret;
    Claw claw;
    Linkage linkage;
    AprilTagDetection_41h12 atag;
    int x = 2;
    @Override
    public void runOpMode() throws InterruptedException {
        atag = new AprilTagDetection_41h12(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        turret = new Turret(this);
        claw = new Claw(this);
        linkage = new Linkage(this);

        Trajectory align_block = drive.trajectoryBuilder(new Pose2d())
                .forward(60) //60
                .build();

        Trajectory go_back = drive.trajectoryBuilder(align_block.end())
                .back(33)
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(go_back.end())
                .strafeLeft(15)
                .forward(5)
                .build();
        TrajectorySequence right = drive.trajectorySequenceBuilder(go_back.end())
                .strafeRight(15)
                .forward(5)
                .build();

        while (!isStarted() && !isStopRequested()) {
            turret.engageBrake();
//            turret.goToAngle(0, 1.0);
        atag.detectZone();
        x = atag.getZone();
        }

        claw.setState(false);
        wait(500);
        linkage.goToLevel(200, 0.2);
        wait(1000);

        drive.followTrajectory(align_block);

        drive.turn(-90);

        claw.setState(true);
        wait(500);

        linkage.goToLevel(350, 0.3);
        wait(5000);
        linkage.goToLevel(200, 0.3);
        wait(1000);

        drive.turn(90);

        drive.followTrajectory(go_back);

        switch(x){
            case 1: drive.followTrajectorySequence(left); break;
            case 2: break;
            case 3: drive.followTrajectorySequence(right); break;
        }
    }
}
