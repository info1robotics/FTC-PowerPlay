package org.firstinspires.ftc.teamcode.OpModes.DriverControl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommonPackage.GamepadEx;
import org.firstinspires.ftc.teamcode.SubSystems.V2.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.V2.Drivetrain;
import org.firstinspires.ftc.teamcode.SubSystems.V2.Linkage;
import org.firstinspires.ftc.teamcode.SubSystems.V2.Turret;

@TeleOp(name = "TeleOp")
public class DriverControl extends LinearOpMode {
    GamepadEx gamepad_2, gamepad_1;
    public double targetAngle, turretVelocity, linkageVelocity, angleThreshold, autoTurretVelocity, autoLinkageVelocity;
    public int targetHeight, heightThreshold;
    public double upperAngleLimit = 360, lowerAngleLimit = -360;
    public int upperLinkageLimit = 470, lowerLinkageLimit = -50;
    public int confirmIncrement = 0;
    public int driver1confirmIncrement = 0;
    public boolean changed;

    @Override
    public void runOpMode() throws InterruptedException {
        Linkage linkage = new Linkage(this);
        Turret turret = new Turret(this);
        Drivetrain drive = new Drivetrain(this.hardwareMap);
        Claw claw = new Claw(this);
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
            drive.vectorMove(gamepad1.left_stick_x, -gamepad1.left_stick_y,
                    gamepad1.right_stick_x + (gamepad1.left_trigger - gamepad1.right_trigger),
//                    gamepad1.right_stick_x,
                    gamepad1.right_bumper ? 0.6 : 0.9);

            if (gamepad2.cross) {
                claw.setSubsystemState(Claw.subsystemStates.READY);
                confirmIncrement = 0;
            }

            if (gamepad2.triangle) {
                claw.setClawState(Claw.clawStates.CLOSED);
                confirmIncrement = 0;
            }

            if (gamepad_2.getButtonDown("x")) {
                if (confirmIncrement == 0) {
                    claw.setSubsystemState(Claw.subsystemStates.EXTENDED);
                    confirmIncrement++;
                } else {
                    targetHeight -= 75;
                    claw.setSubsystemState(Claw.subsystemStates.EXTENDED_DROP);
                    claw.setClawState(Claw.clawStates.OPEN);
                    confirmIncrement = 0;
                }
            }
            if (gamepad2.right_stick_button) {
                targetAngle = 180;
            }
            if (gamepad2.left_stick_button) {
                targetAngle = -180;
            }
            if (gamepad2.circle) {
                targetAngle = 0;
            }
            if (gamepad1.cross) {
                turret.hardReset();
                targetAngle = 0;
            }

            // auto collect from substation

            if (gamepad2.left_stick_y > 0.8) {
                targetAngle = 0;
                targetHeight = -50;
                claw.setSubsystemState(Claw.subsystemStates.READY);
            }

            // auto align high

            if (gamepad2.left_stick_y < -0.8) {
                claw.setSubsystemState(Claw.subsystemStates.EXTENDED);
                targetAngle = 180;
                targetHeight = 430;
                confirmIncrement = 1;
            }

            // auto align mid

            if (gamepad2.right_stick_y < -0.8) {
                claw.setSubsystemState(Claw.subsystemStates.EXTENDED);
                targetAngle = 180;
                targetHeight = 235;
                confirmIncrement = 1;
            }

            // auto align low

            if (gamepad2.dpad_up) {
                targetHeight = 100;
                claw.setSubsystemState(Claw.subsystemStates.EXTENDED);
                confirmIncrement = 1;
            }

            // auto align ground

            if (gamepad2.dpad_down) {
                targetHeight = 30;
                claw.setPivotPosition(Claw.pivotPositions.DOWN);
                claw.setClawState(Claw.clawStates.CLOSED);
                confirmIncrement = 1;
            }

            if (gamepad2.left_bumper) targetHeight -= linkage.THRESHOLD;
            if (gamepad2.right_bumper) targetHeight += linkage.THRESHOLD;
            if (gamepad2.left_trigger > 0.1) targetAngle += angleThreshold;
            if (gamepad2.right_trigger > 0.1) targetAngle -= angleThreshold;

            if (targetHeight > upperLinkageLimit) targetHeight = upperLinkageLimit;
            if (targetHeight < lowerLinkageLimit) targetHeight = lowerLinkageLimit;

            if (targetAngle > upperAngleLimit) targetAngle = upperAngleLimit;
            if (targetAngle < lowerAngleLimit) targetAngle = lowerAngleLimit;

            linkage.setHeight(targetHeight, linkageVelocity);
            turret.setHeading(targetAngle, turretVelocity);
            gamepad_2.update();
            telemetry.update();
        }
    }
}
