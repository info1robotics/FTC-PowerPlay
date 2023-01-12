package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.SubSystems.Linkage.CURRENT_LEVEL;
import static org.firstinspires.ftc.teamcode.SubSystems.Linkage.GROUND_LEVEL;
import static org.firstinspires.ftc.teamcode.SubSystems.Linkage.LINKAGE_MAX;
import static org.firstinspires.ftc.teamcode.SubSystems.Linkage.LINKAGE_MIN;
import static org.firstinspires.ftc.teamcode.SubSystems.Linkage.LINKAGE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.SubSystems.Linkage.LOW_LEVEL;
import static org.firstinspires.ftc.teamcode.SubSystems.Linkage.MID_LEVEL;
import static org.firstinspires.ftc.teamcode.SubSystems.Linkage.HIGH_LEVEL;
import static org.firstinspires.ftc.teamcode.SubSystems.Linkage.SAFETY_THRESHOLD;
import static org.firstinspires.ftc.teamcode.SubSystems.Turret.CURRENT_ANGLE;
import static org.firstinspires.ftc.teamcode.SubSystems.Turret.ANGLE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.SubSystems.Turret.CURRENT_TICK;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Common.Mecanum;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.Linkage;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

@TeleOp(name = "StandardTeleOp")
public class  StardardTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize all systems.
        Linkage linkage = new Linkage(this);
        Claw claw = new Claw(this);
        Turret turret = new Turret(this);
        Mecanum drivetrain = new Mecanum(hardwareMap);

        // Fire up the motors.
        CURRENT_LEVEL = 0;
        CURRENT_ANGLE = 0;
        linkage.GO_TO_LEVEL(CURRENT_LEVEL, 1.0);
        turret.GO_TO_ANGLE(CURRENT_ANGLE, 1.0);
        boolean STATE_CHANGED = false;

        waitForStart();
        while(opModeIsActive()){
            // Omnidirectional drivetrain control.
            drivetrain.vectorMove(
                    -gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    gamepad1.left_trigger - gamepad1.right_trigger,
                    gamepad1.right_bumper ? 0.5 : 1.0
            );

//           Toggle claw power when button A (Xbox) / X (PS4) is pressed.
            if(gamepad2.a && !STATE_CHANGED) {claw.TOGGLE(); STATE_CHANGED = true;}
            else if(!gamepad2.a) STATE_CHANGED = false;

            // Set desired linkage height using fixed counts on the left d-pad.
            if(gamepad2.dpad_up) CURRENT_LEVEL = HIGH_LEVEL;
            if(gamepad2.dpad_right) CURRENT_LEVEL = MID_LEVEL;
            if(gamepad2.dpad_left) CURRENT_LEVEL = LOW_LEVEL;
            if(gamepad2.dpad_down) CURRENT_LEVEL = GROUND_LEVEL;

            // Fine-tune the current height on the linkage system.
            if(gamepad2.left_bumper) CURRENT_LEVEL -= LINKAGE_THRESHOLD;
            if(gamepad2.right_bumper) CURRENT_LEVEL += LINKAGE_THRESHOLD;

            // Reset turret angle to 0 when the X (Xbox) / Square (PS4) button is pressed for safe linkage operation.
            if(CURRENT_LEVEL > SAFETY_THRESHOLD && gamepad2.circle) CURRENT_ANGLE = 0;

            // Manually tune the turret angle. The turret will not spin if it is currently at risk of damaging the linkage.
            if(CURRENT_LEVEL > SAFETY_THRESHOLD && gamepad2.left_trigger!=0) CURRENT_ANGLE += ANGLE_THRESHOLD;
            if(CURRENT_LEVEL > SAFETY_THRESHOLD && gamepad2.right_trigger!=0) CURRENT_ANGLE -= ANGLE_THRESHOLD;

            telemetry.addData("Linkage: ", linkage.linkageLeft.getCurrentPosition());

            if(CURRENT_LEVEL > LINKAGE_MAX) CURRENT_LEVEL = LINKAGE_MAX;
            if(CURRENT_LEVEL < LINKAGE_MIN) CURRENT_LEVEL = LINKAGE_MIN;

            // Apply settings to motors.
            linkage.GO_TO_LEVEL(CURRENT_LEVEL, 1.0);
            turret.GO_TO_ANGLE(CURRENT_ANGLE, 1.0);
            telemetry.addData("ticks",turret.turretMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
