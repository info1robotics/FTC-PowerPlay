package org.firstinspires.ftc.teamcode.OpModes.V3;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommonPackage.GamepadEx;
import org.firstinspires.ftc.teamcode.SubSystems.V3.Turret;

@TeleOp
public class TurretDebug extends LinearOpMode {

    GamepadEx gamepad_2, gamepad_1;
    public double targetAngle, turretVelocity, linkageVelocity, angleThreshold, autoTurretVelocity, autoLinkageVelocity;
    public int targetHeight, heightThreshold;
    public double upperAngleLimit = 360, lowerAngleLimit = -360;
    public int upperLinkageLimit = 550, lowerLinkageLimit = -40;
    public int confirmIncrement = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Turret turret = new Turret(this);
        gamepad_2 = new GamepadEx(gamepad2);
        gamepad_1 = new GamepadEx(gamepad1);


        targetAngle = 0.0;
        targetHeight = 0;

        turretVelocity = 0.65;
        linkageVelocity = 0.65;

        autoTurretVelocity = 1.0;
        autoLinkageVelocity = 1.0;

        heightThreshold = 1;
        angleThreshold = 1.0;

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.dpad_right) {
                targetAngle = 180;
            }
            if (gamepad2.dpad_left) {
                targetAngle = -180;
            }
            if (gamepad2.circle) {
                targetAngle = 0;
            }
            if (gamepad1.cross) {
                turret.hardReset();
                targetAngle = 0;
            }

            if (gamepad2.right_stick_x < -0.8) {
                targetAngle = -90;
            }

            if (gamepad2.right_stick_x > 0.8) {
                targetAngle = 90;
            }


            if (gamepad2.left_trigger > 0.1) targetAngle += angleThreshold;
            if (gamepad2.right_trigger > 0.1) targetAngle -= angleThreshold;

            if (targetAngle > upperAngleLimit) targetAngle = upperAngleLimit;
            if (targetAngle < lowerAngleLimit) targetAngle = lowerAngleLimit;

            turret.setHeading(targetAngle, turretVelocity);
//            turret.debug();
            telemetry.addData("Turret's Current Angle Heading ", turret.getCurrentAngleHeading());
            telemetry.addData("Turret's Current Tick Count ", turret.turretMotor.getCurrentPosition());
            gamepad_2.update();
            telemetry.update();
        }
    }
}
