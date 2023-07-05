package org.firstinspires.ftc.teamcode.OpModes.V3;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CommonPackage.GamepadEx;
import org.firstinspires.ftc.teamcode.SubSystems.V2.Drivetrain;
import org.firstinspires.ftc.teamcode.SubSystems.V3.Controller;
import org.firstinspires.ftc.teamcode.SubSystems.V3.Lift;

import java.util.LinkedList;
import java.util.Queue;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
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
    public int targetHeight, heightThreshold, maxLift, minLift;
    public double upperAngleLimit = 360, lowerAngleLimit = -360;
    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drive = new Drivetrain(this.hardwareMap);
        Controller controller = new Controller(this);
        controller.lift.resetEncoders();
        controller.claw.open();
        controller.clawFlip.setCollect();
        controller.pivot.setCollect();
        
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

        minLift = 0;
        maxLift = 1800;

        waitForStart();

        new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                drive.vectorMove(-gamepad1.left_stick_x, gamepad1.left_stick_y,
                        (gamepad1.left_trigger - gamepad1.right_trigger),
                        gamepad1.right_bumper ? 0.6 : 1.0);
                gamepadEx1.update();
            }
        }).start();

        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad2.dpad_up) {
                targetHeight = Lift.HIGH_POS;
            }
            if (gamepad2.dpad_left) {
                targetHeight = Lift.MID_POS;
            }
            if (gamepad2.dpad_right) {
                targetHeight = Lift.LOW_POS;
            }
            if (gamepad2.dpad_down) {
                targetHeight = Lift.COLLECT_POS;
            }

            if (gamepad2.circle) {
                controller.pivot.setCollect();
                controller.clawFlip.setCollect();
            }

            if (gamepad2.square) {
                new Thread(() -> {
                    controller.pivot.setScore();
                    sleep(550);
                    controller.clawFlip.setScore();
                }).start();
            }


            if (gamepadEx2.getButtonDown("a")) {
                controller.claw.toggle();
            }

            if(gamepadEx2.getButtonDown("bumper_right")) targetHeight -= 500;

            if(targetHeight < minLift) targetHeight = minLift;
            if(targetHeight > maxLift) targetHeight = maxLift;

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
