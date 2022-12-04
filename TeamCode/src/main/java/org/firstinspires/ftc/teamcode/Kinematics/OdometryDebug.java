package org.firstinspires.ftc.teamcode.Kinematics;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Common.GamepadEx;
import org.firstinspires.ftc.teamcode.Common.Mecanum;

@Disabled
@TeleOp(name = "Odometry Pod Debug")
public class OdometryDebug extends LinearOpMode {

    DcMotor odoLeft,odoRight,odoCenter;
    static final double WHEEL_DIAMETER = 2;
    static final double WHEEL_DIAMETER_TO_NORMAL_UNITS = WHEEL_DIAMETER * 2.54;
    static final double WHEEL_RADIUS = WHEEL_DIAMETER_TO_NORMAL_UNITS / 2;
    static final double WHEEL_CIRCUMFERENCE = Math.PI * 2 * WHEEL_RADIUS;
    static final double TICKS_PER_REVOLUTION = 8192;
    static double LEFT_ODOMETRY_DISTANCE_TRAVELED;
    static double RIGHT_ODOMETRY_DISTANCE_TRAVELED;
    static double CENTER_ODOMETRY_DISTANCE_TRAVELED;

    @Override
    public void runOpMode() throws InterruptedException {

        Mecanum mecanum = new Mecanum(hardwareMap);
        GamepadEx g1 = new GamepadEx(gamepad1);

        odoLeft = hardwareMap.get(DcMotor.class, "OdometryLeft");
        odoRight = hardwareMap.get(DcMotor.class, "OdometryRight");
        odoCenter = hardwareMap.get(DcMotor.class, "OdometryBack");

        odoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        odoRight.setDirection(DcMotorSimple.Direction.REVERSE);

        OdometrySystem odo = new OdometrySystem(hardwareMap);

        waitForStart();

        while(opModeIsActive()){

            g1.update();
            mecanum.vectorMove(
                    -gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    (gamepad1.left_trigger - gamepad1.right_trigger),
                    gamepad1.right_bumper ? 0.5 : 1.0);

            LEFT_ODOMETRY_DISTANCE_TRAVELED = (odoLeft.getCurrentPosition() / TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE;
            RIGHT_ODOMETRY_DISTANCE_TRAVELED = (odoRight.getCurrentPosition() / TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE;
            CENTER_ODOMETRY_DISTANCE_TRAVELED = (odoCenter.getCurrentPosition() / TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE;

            telemetry.addData("Left Odometry Pod Raw Value ",odoLeft.getCurrentPosition());
            telemetry.addData("Left Odometry Pod Distance Traveled (cm)",LEFT_ODOMETRY_DISTANCE_TRAVELED);

            telemetry.addLine("");

            telemetry.addData("Right Odometry Pod Raw Value ",odoRight.getCurrentPosition());
            telemetry.addData("Right Odometry Pod Distance Traveled (cm)",RIGHT_ODOMETRY_DISTANCE_TRAVELED);

            telemetry.addLine("");

            telemetry.addData("Center Odometry Pod Raw Value ",odoCenter.getCurrentPosition());
            telemetry.addData("Center Odometry Pod Distance Traveled (cm)",CENTER_ODOMETRY_DISTANCE_TRAVELED);

            telemetry.addLine("");

            telemetry.addData("Estimated ", odo.getCurrentHeading());

            telemetry.update();
        }
    }
}
