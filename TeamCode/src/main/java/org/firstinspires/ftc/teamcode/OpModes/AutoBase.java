package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.SubSystems.Linkage.DESIRED_HEIGHT;
import static org.firstinspires.ftc.teamcode.SubSystems.Turret.CORRECTED_ANGLE;
import static org.firstinspires.ftc.teamcode.SubSystems.Turret.DESIRED_ANGLE;
import static org.firstinspires.ftc.teamcode.SubSystems.Turret.AUTO_SPEED;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
//    public Pose2d startPose;
    public Linkage linkage;
    public Claw claw;

//    public AprilTagDetection_41h12 atag;
    public int x = 0;
    @Override
    public final void runOpMode() throws InterruptedException {
        claw = new Claw(this);
        turret = new Turret(this);
        linkage = new Linkage(this);
//        atag = new AprilTagDetection_41h12(this);
        drive = new SampleMecanumDrive(hardwareMap);
//        startPose = new Pose2d(-37.5, -62, Math.toRadians(90));
//        drive.setPoseEstimate(startPose);

        DESIRED_ANGLE = 0;
        CORRECTED_ANGLE = 0;
        AUTO_SPEED = 0.2;
        turret.engageBrake();
        turret.engageSuperBrake();

        DESIRED_HEIGHT = 0;
        onInit();
        while (!isStarted() && !isStopRequested()) {
//            atag.detectZone();
//            x = atag.getZone();
//            telemetry.update();
        }
        task.start(this);
        while(opModeIsActive() && task.isRunning()) {
            linkage.update();
            turret.update();
            telemetry.addData("distance value", turret.distanceSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("angle fed into motor", CORRECTED_ANGLE);
            onLoop();
            telemetry.update();
            task.tick();
        }
    }

    public void onInit() {};
    public void onStart() {};
    public void onLoop() {};
}
