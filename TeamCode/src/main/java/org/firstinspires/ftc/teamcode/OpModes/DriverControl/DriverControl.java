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
    public int upperLinkageLimit = 440, lowerLinkageLimit = -50;
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
        claw.setSubsystemState(Claw.subsystemStates.RETRACTED);

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
            drive.vectorMove(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x + (gamepad1.left_trigger - gamepad1.right_trigger),
//                    gamepad1.right_stick_x,
                    gamepad1.right_bumper ? 0.75 : 1.0);

            if (gamepad2.cross) {
                claw.setSubsystemState(Claw.subsystemStates.READY);
                confirmIncrement = 0;

            }

            if (gamepad2.triangle) {
                claw.setSubsystemState(Claw.subsystemStates.COLLECTED);
                confirmIncrement = 0;
            }

            if (gamepad_2.getButtonDown("x")) {
                if(confirmIncrement == 0){
                    claw.setPivotPosition(Claw.pivotPositions.INTERMEDIARY);
                    confirmIncrement++;
                } else {
                    claw.setPivotPosition(Claw.pivotPositions.INTERMEDIARY);
                    claw.setClawState(Claw.clawStates.OPEN);
                    confirmIncrement = 0;
                }
            }

//            if (gamepad_1.getButtonDown("y")) {
//                if(driver1confirmIncrement == 0){
//                    claw.setPivotPosition(Claw.pivotPositions.DOWN);
//                    claw.setClawState(Claw.clawStates.CLOSED);
//                    driver1confirmIncrement++;
//                } else {
//                    claw.setPivotPosition(Claw.pivotPositions.DOWN);
//                    claw.setClawState(Claw.clawStates.OPEN);
//                    driver1confirmIncrement = 0;
//                }
//            }

            if (gamepad2.right_stick_button) {
                targetAngle = -180;
            }
            if (gamepad2.left_stick_button) {
                targetAngle = 180;
            }
            if (gamepad2.circle) {
                targetAngle = 0;
            }
            if (gamepad2.dpad_up) {
                targetHeight = 400;
            }
            if (gamepad2.dpad_left) {
                targetHeight = 275;
            }
            if (gamepad2.dpad_down) {
                targetHeight = 150;
            }
            if  (gamepad1.cross){
                turret.hardReset();
                targetAngle = 0;
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
