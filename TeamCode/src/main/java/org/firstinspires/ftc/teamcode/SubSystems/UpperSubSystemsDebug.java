package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.teamcode.SubSystems.Linkage.CURRENT_LEVEL;
import static org.firstinspires.ftc.teamcode.SubSystems.Linkage.LINKAGE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.SubSystems.Linkage.SAFETY_THRESHOLD;
import static org.firstinspires.ftc.teamcode.SubSystems.Turret.ANGLE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.SubSystems.Turret.CURRENT_ANGLE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Common.GamepadEx;
@Disabled
@TeleOp(name = "UpperSubSystemsDebug")
public class UpperSubSystemsDebug extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize all systems.
        Turret turret = new Turret(this);
        Linkage linkage = new Linkage(this);
        GamepadEx gamepadEx = new GamepadEx(gamepad2);

        // Fire up the motors.
        CURRENT_LEVEL = 0;
        CURRENT_ANGLE = 0;
        linkage.goToLevel(CURRENT_LEVEL, 1.0);
        turret.goToAngle(CURRENT_ANGLE, 1.0);
        boolean BRAKE_CHANGED = false;

        turret.disengageBrake();
        waitForStart();

        while(opModeIsActive()){
            if(gamepad2.square && !BRAKE_CHANGED) {turret.toggleBrake(); BRAKE_CHANGED = true;}
            else if(!gamepad2.square) BRAKE_CHANGED = false;

            if(gamepad2.left_bumper) CURRENT_LEVEL -= LINKAGE_THRESHOLD;
            if(gamepad2.right_bumper) CURRENT_LEVEL += LINKAGE_THRESHOLD;

            if(CURRENT_LEVEL > SAFETY_THRESHOLD && gamepad2.left_trigger!=0) CURRENT_ANGLE += ANGLE_THRESHOLD;
            if(CURRENT_LEVEL > SAFETY_THRESHOLD && gamepad2.right_trigger!=0) CURRENT_ANGLE -= ANGLE_THRESHOLD;

            if (gamepadEx.getButtonUp("dpad_up")) {
                turret.setPower(.5);
                turret.turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else if (gamepadEx.getButtonDown("dpad_up")) {
                turret.setMotorsRunMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setTargetPosition(turret.turretMotor.getCurrentPosition());
            }

            if (gamepadEx.getButtonUp("dpad_down")) {
                turret.setPower(-.5);
                turret.turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else if (gamepadEx.getButtonDown("dpad_down")) {
                turret.setMotorsRunMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setTargetPosition(turret.turretMotor.getCurrentPosition());
            }

            if(!gamepad2.dpad_up)   linkage.goToLevel(CURRENT_LEVEL, 0.8);
            turret.goToAngle(CURRENT_ANGLE, 0.8);
            telemetry.update();
        }
    }
}
