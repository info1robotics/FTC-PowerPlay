package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.EOCV.f41h12.AprilTagDetection_41h12;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.Linkage;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.Tasks.Task;

public abstract class AutoOpMode extends LinearOpMode {
    public Task task;
    public SampleMecanumDrive drive;
    public Turret turret;
    public Pose2d startPose;
    public Linkage linkage;
    public Claw claw;
    public AprilTagDetection_41h12 atag;
    public int x = 0;
    @Override
    public final void runOpMode() throws InterruptedException {
        claw = new Claw(this);
        turret = new Turret(this);
        linkage = new Linkage(this);
        atag = new AprilTagDetection_41h12(this);
        drive = new SampleMecanumDrive(hardwareMap);
        startPose = new Pose2d(-35, -62, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        turret.engageBrake();
        turret.goToAngle(0,1.0);
        linkage.goToLevel(0, 1.0);
        claw.setState(true);
        onInit();
        while (!isStarted() && !isStopRequested()) {
            atag.detectZone();
            x = atag.getZone();
            telemetry.addData("x = ", x);
            telemetry.update();
        }
        waitForStart();
        task.start(this);
        while(opModeIsActive() && task.isRunning()) {
            onLoop();
            task.tick();
        }
    }

    public void onInit() {};
    public void onStart() {};
    public void onLoop() {};
}
