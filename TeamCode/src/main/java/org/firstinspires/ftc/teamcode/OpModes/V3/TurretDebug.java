package org.firstinspires.ftc.teamcode.OpModes.V3;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommonPackage.GamepadEx;
import org.firstinspires.ftc.teamcode.SubSystems.V2.Drivetrain;
import org.firstinspires.ftc.teamcode.SubSystems.V3.Controller;
import org.firstinspires.ftc.teamcode.SubSystems.V3.Extendo;
import org.firstinspires.ftc.teamcode.SubSystems.V3.Lift;

import java.util.LinkedList;
import java.util.Queue;

@TeleOp
public class TurretDebug extends LinearOpMode {
    Queue<Pair<Runnable, Long>> actionQueue = new LinkedList<>();

    public void updateQueue() {
        if (actionQueue.isEmpty()) return;
        Pair<Runnable, Long> action = actionQueue.peek();
        if (action.second < System.currentTimeMillis()) {
            actionQueue.remove();
            action.first.run();
        }
    }

    GamepadEx gamepadEx1, gamepadEx2;
    public double targetAngle, turretVelocity, linkageVelocity, angleThreshold, autoTurretVelocity, autoLinkageVelocity;
    public int targetHeight, heightThreshold;
    public double upperAngleLimit = 360, lowerAngleLimit = -360;
    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drive = new Drivetrain(this.hardwareMap);
        Controller controller = new Controller(this);
        controller.lift.resetEncoders();
        controller.claw.open();
        
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        targetAngle = 0.0;
        targetHeight = 0;

        turretVelocity = 0.65;
        linkageVelocity = 0.65;

        autoTurretVelocity = 1.0;
        autoLinkageVelocity = 1.0;

        heightThreshold = 1;
        angleThreshold = 1.0;

        waitForStart();

        new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                drive.vectorMove(gamepad2.left_stick_x, gamepad2.left_stick_y,
                        (gamepad2.left_trigger - gamepad2.right_trigger),
                        gamepad2.right_bumper ? 0.6 : 1.0);
                gamepadEx1.update();
            }
        }).start();

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad2.dpad_right) {
                targetAngle = 180;
            }
            if (gamepad2.dpad_left) {
                targetAngle = -180;
            }
            if (gamepad2.circle) {
                targetAngle = 0;
            }
//            if (gamepad1.cross) {
//                controller.turret.hardReset();
//                targetAngle = 0;
//            }

            if (gamepadEx2.getButtonDown("b")) {
                controller.togglePivotAndClawFlip();
            }

            if (gamepadEx2.getButtonDown("a")) {
                controller.claw.toggle();
            }

//            if (gamepad2.right_stick_x < -0.8) {
//                targetAngle = -90;
//            }
//
//            if (gamepad2.right_stick_x > 0.8) {
//                targetAngle = 90;
//            }

            if (gamepad2.right_stick_y > 0.8) {
                targetHeight = Lift.LOW_POS;
            }

            if (gamepad2.right_stick_x < -0.8) {
                targetHeight = Lift.MID_POS;
            }

            if (gamepad2.right_stick_y < -0.8) {
                targetHeight = Lift.HIGH_POS;
            }


//            if (gamepad2.left_trigger > 0.1) targetAngle += angleThreshold;
//            if (gamepad2.right_trigger > 0.1) targetAngle -= angleThreshold;

            if (targetAngle > upperAngleLimit) targetAngle = upperAngleLimit;
            if (targetAngle < lowerAngleLimit) targetAngle = lowerAngleLimit;

            controller.turret.setHeading(targetAngle, turretVelocity);
            controller.lift.setHeight(targetHeight, 1);
            telemetry.addData("Turret's Current Angle Heading ", controller.turret.getCurrentAngleHeading());
            telemetry.addData("Turret's Current Tick Count ", controller.turret.turretMotor.getCurrentPosition());

            updateQueue();
            gamepadEx2.update();
            telemetry.update();
        }
    }
}
