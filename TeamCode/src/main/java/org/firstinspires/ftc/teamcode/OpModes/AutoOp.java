package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.sequencesegment.TrajectorySegment;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

@Autonomous
public class AutoOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Turret turret = new Turret(this);
        Pose2d startPose = new Pose2d(-35, -62, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        TrajectorySequence myTrajectory = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-35, 0))
                .build();
        turret.engageBrake();
        turret.goToAngle(0,1.0);
        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(myTrajectory);
        turret.disengageBrake();
    }
}
