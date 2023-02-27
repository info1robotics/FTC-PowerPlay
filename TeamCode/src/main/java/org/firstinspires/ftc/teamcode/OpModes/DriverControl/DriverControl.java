package org.firstinspires.ftc.teamcode.OpModes.DriverControl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.V2.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.V2.Drivetrain;
import org.firstinspires.ftc.teamcode.SubSystems.V2.Linkage;
import org.firstinspires.ftc.teamcode.SubSystems.V2.Turret;

@TeleOp(name = "TeleOp")
public class DriverControl extends LinearOpMode {
    public double targetAngle, turretVelocity, linkageVelocity, angleThreshold, autoTurretVelocity, autoLinkageVelocity;
    public int targetHeight, heightThreshold;
    public double upperAngleLimit = 360, lowerAngleLimit = -360;
    public int upperLinkageLimit = 440, lowerLinkageLimit = -20;

    @Override
    public void runOpMode() throws InterruptedException {
        Linkage linkage = new Linkage(this);
        Turret turret = new Turret(this);
        Drivetrain drive = new Drivetrain(this.hardwareMap);
        Claw claw = new Claw(this);

        claw.setSubsystemState(Claw.subsystemStates.RETRACTED);

        targetAngle = 0.0;
        targetHeight = 0;

        turretVelocity = 0.65;
        linkageVelocity = 0.65;

        autoTurretVelocity = 1.0;
        autoLinkageVelocity = 1.0;

        heightThreshold = 1;
        angleThreshold = 2.0;

        waitForStart();
        while (opModeIsActive()) {
            drive.vectorMove(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x + (gamepad1.left_trigger - gamepad1.right_trigger),
//                    gamepad1.right_stick_x,
                    gamepad1.right_bumper ? 0.75 : 1.0);

            if (gamepad2.cross) {
                claw.setSubsystemState(Claw.subsystemStates.READY);
            }

            if (gamepad2.triangle) {
                claw.setSubsystemState(Claw.subsystemStates.COLLECTED);
            }

            if (gamepad2.square) {
                claw.setSubsystemState(Claw.subsystemStates.DROP);
            }

            if (gamepad2.right_stick_button) {
                targetAngle = -90;
            }
            if (gamepad2.left_stick_button) {
                targetAngle = 90;
            }
            if (gamepad2.circle) {
                targetAngle = 0;
            }
            if (gamepad2.touchpad) {
                targetAngle = 180;
            }
            if (gamepad2.dpad_up) {
                targetHeight = 400;
            }
            if  (gamepad1.x){
                turret.hardReset();
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
            telemetry.update();
        }

    }
}
