package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.CommonPackage.AutoUtils;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.V3.Controller;
import org.firstinspires.ftc.teamcode.Tasks.Task;

public abstract class AutoBase extends LinearOpMode {
    public Task task;
    public SampleMecanumDrive drive;
    public Controller ct;
    public double linkageVelocity = 1.0;
    public int targetHeight = 0, preferredZone = 0;
//    public AprilTagDetection_41h12 atag;

    public String junction = "";
    public boolean lockOnJunction = false;

    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    @Override
    public final void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ct = new Controller(this);
        drive = new SampleMecanumDrive(hardwareMap);
//        atag = new AprilTagDetection_41h12(this);

        ct.lift.resetEncoders();

        ct.turret.resetEncoder();
        ct.turret.setVelocity(1.0);
        ct.turret.setTargetAngle(0.0);

        onInit();
        SampleMecanumDrive.imu.startIMUThread(this);
        while (!isStarted() && !isStopRequested()) {
            ct.setScorePivotAndClawFlip();
            ct.turret.update();
            ct.lift.setHeight(0, linkageVelocity);
//            atag.detectZone();
//            preferredZone = atag.getZone();
            telemetry.addData("Voltage", getBatteryVoltage());
            telemetry.addData("IMU Heading", Math.toDegrees(drive.getExternalHeading()));
            telemetry.addData("IMU Heading Raw", Math.toDegrees(drive.getRawExternalHeading()));
            telemetry.update();
        }

//        atag.getAprilTagDetectionPipeline().finalize();
//        atag.getCamera().setPipeline(null);
//        atag.getCamera().closeCameraDeviceAsync(() -> {});

        task.start(this);

        while(opModeIsActive()) {
            if (isStopRequested()) break;
            telemetry.addData("Voltage", getBatteryVoltage());
            task.tick();
            ct.lift.setHeight(targetHeight, linkageVelocity);

            if (lockOnJunction && !junction.equals("")) {
                Pose2d normalisedPosition = AutoUtils.convertToNormalAxis(drive.getPoseEstimate());
                ct.turret.lockOnJunction(junction, normalisedPosition);
//                ct.turret.setHeading((Turret.targetAngle), .6);
            }
            telemetry.addData("Turret Velo", ct.turret.turretMotor.getPower());
            ct.turret.update();
            drive.update();
            telemetry.update();
        }
//        atag.getCamera().closeCameraDevice();
    }

    public void onInit() {};
    public void onStart() {};
    public void onLoop() {};
}
