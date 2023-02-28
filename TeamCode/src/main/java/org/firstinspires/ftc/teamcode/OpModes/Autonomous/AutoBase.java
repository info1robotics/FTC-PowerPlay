package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.EOCV.f41h12.AprilTagDetection_41h12;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.V2.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.V2.Linkage;
import org.firstinspires.ftc.teamcode.SubSystems.V2.Turret;
import org.firstinspires.ftc.teamcode.Tasks.Task;

public abstract class AutoBase extends LinearOpMode {
    public Task task;
    public SampleMecanumDrive drive;
    public Turret turret;
    public Linkage linkage;
    public Claw claw;
    public double linkageVelocity = 1.0;
    public int targetHeight = 0, preferredZone = 0;
    public AprilTagDetection_41h12 atag;
    @Override
    public final void runOpMode() throws InterruptedException {
        claw = new Claw(this);
        turret = new Turret(this);
        linkage = new Linkage(this);
        drive = new SampleMecanumDrive(hardwareMap);
        atag = new AprilTagDetection_41h12(this);

        turret.resetEncoder();
        linkage.resetEncoders();
        turret.setTurretVelocity(1.0);
        turret.setTargetAngle(0.0);
        claw.setSubsystemState(Claw.subsystemStates.RETRACTED);

        onInit();
        while (!isStarted()) {
            atag.detectZone();
            preferredZone = atag.getZone();
            telemetry.update();
        }

        task.start(this);
        while(opModeIsActive() && task.isRunning()) {
            onLoop();
            turret.update();
            linkage.setHeight(targetHeight, linkageVelocity);
            telemetry.update();
            task.tick();
        }
    }

    public void onInit() {};
    public void onStart() {};
    public void onLoop() {};
}
