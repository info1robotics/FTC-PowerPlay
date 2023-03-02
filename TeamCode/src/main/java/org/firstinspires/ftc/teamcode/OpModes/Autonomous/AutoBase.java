package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.EOCV.f41h12.AprilTagDetection_41h12;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.V2.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.V2.Linkage;
import org.firstinspires.ftc.teamcode.SubSystems.V2.Turret;
import org.firstinspires.ftc.teamcode.Tasks.Task;
import org.openftc.easyopencv.OpenCvCamera;

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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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
        while (!isStarted() && !isStopRequested()) {
            atag.detectZone();
            preferredZone = atag.getZone();
            telemetry.update();
        }

//        atag.getCamera().setPipeline(null);

        task.start(this);

        while(opModeIsActive()) {
            if(isStopRequested()) break;
            task.tick();
            turret.update();
            linkage.setHeight(targetHeight, linkageVelocity);
            telemetry.update();
        }
    }

    public void onInit() {};
    public void onStart() {};
    public void onLoop() {};
}
