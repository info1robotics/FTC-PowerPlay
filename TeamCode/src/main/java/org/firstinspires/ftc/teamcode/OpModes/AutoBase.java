package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.EOCV.f41h12.AprilTagDetection_41h12;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.Linkage;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.Tasks.Task;

public abstract class AutoBase extends LinearOpMode {
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
        turret.engageSuperBrake();
        turret.goToAngle(0,1.0);
        linkage.goToLevel(0, 1.0);
        onInit();
        while (!isStarted() && !isStopRequested()) {
            atag.detectZone();
            x = atag.getZone();
            telemetry.update();
        }
        task.start(this);
        while(opModeIsActive() && task.isRunning()) {
            onLoop();
            linkage.debug();
            telemetry.update();
            task.tick();
        }
    }

    public void onInit() {};
    public void onStart() {};
    public void onLoop() {};
}
