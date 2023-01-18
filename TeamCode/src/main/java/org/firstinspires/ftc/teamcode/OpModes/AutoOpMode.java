package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
    @Override
    public final void runOpMode() throws InterruptedException {
        claw = new Claw(this);
        turret = new Turret(this);
        linkage = new Linkage(this);
        startPose = new Pose2d(-35, -62, Math.toRadians(90));
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        turret.engageBrake();
        turret.goToAngle(0,1.0);
        linkage.goToLevel(0, 1.0);
        claw.setState(false);
        onInit();
        waitForStart();
        onStart();
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
