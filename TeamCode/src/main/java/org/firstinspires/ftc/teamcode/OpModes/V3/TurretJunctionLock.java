package org.firstinspires.ftc.teamcode.OpModes.V3;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommonPackage.ThreadedIMU;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.SubSystems.V2.Drivetrain;
import org.firstinspires.ftc.teamcode.SubSystems.V3.Turret;

@TeleOp
@Config
public class TurretJunctionLock extends LinearOpMode {
    public static String junction = "B3";
    public static double X = -88.27;
    public static double Y = 59.5;

    public Pose2d convertToNormalAxis(Pose2d old) {
        return new Pose2d(-old.getY(), old.getX());
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap);
        Drivetrain dt = new Drivetrain(this.hardwareMap);
        Turret turret = new Turret(this);

        SampleMecanumDrive.imu.startIMUThread(this);
        drive.setPoseEstimate(new Pose2d(X, Y));

        waitForStart();
        while (!isStopRequested() && isStarted()) {
            dt.vectorMove(-gamepad1.left_stick_x, gamepad1.left_stick_y,
                    (gamepad1.left_trigger - gamepad1.right_trigger),
                    gamepad1.right_bumper ? 0.6 : 1.0);

            Pose2d normalPos = convertToNormalAxis(drive.getPoseEstimate());
            telemetry.addData("Pose Estimate X", normalPos.getX());
            telemetry.addData("Pose Estimate Y", normalPos.getY());
            telemetry.addData("Target X", Turret.parseCoordinates(junction)[0]);
            telemetry.addData("Target Y", Turret.parseCoordinates(junction)[1]);

            turret.lockOnJunction(junction, normalPos, 0);

            telemetry.addData("Target Angle", Turret.targetAngle);
            telemetry.addData("Current Angle", turret.getCurrentAngleHeading());
            telemetry.addData("Robot Heading", drive.getRawExternalHeading());
            telemetry.addData("IMU Raw", SampleMecanumDrive.imu.getHeading());
            telemetry.addData("IMU Deg", Math.toDegrees(SampleMecanumDrive.imu.getHeading()));
            telemetry.update();
            turret.setHeading((Turret.targetAngle), 1);
            drive.update();
        }
    }
}
