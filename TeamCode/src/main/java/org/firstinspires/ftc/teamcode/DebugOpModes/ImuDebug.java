package org.firstinspires.ftc.teamcode.DebugOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Autonomous
public class ImuDebug extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap);
        SampleMecanumDrive.imu.startIMUThread(this);
        IMU imuTurret = hardwareMap.get(IMU.class, "turretIMU");
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Raw Heading Robot", Math.toDegrees(drive.getRawExternalHeading()));
            telemetry.addData("RR Heading Robot", Math.toDegrees(drive.getExternalHeading()));
            telemetry.addData("Raw Heading Turret", Math.toDegrees(imuTurret.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
            telemetry.update();
        }
    }
}
