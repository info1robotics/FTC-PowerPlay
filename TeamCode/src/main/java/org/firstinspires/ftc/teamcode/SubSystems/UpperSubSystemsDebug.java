package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.teamcode.SubSystems.Linkage.CURRENT_LEVEL;
import static org.firstinspires.ftc.teamcode.SubSystems.Linkage.LINKAGE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.SubSystems.Linkage.SAFETY_THRESHOLD;
import static org.firstinspires.ftc.teamcode.SubSystems.Turret.ANGLE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.SubSystems.Turret.CURRENT_ANGLE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "UpperSubSystemsDebug")
public class UpperSubSystemsDebug extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize all systems.
        Turret turret = new Turret(this);
        Linkage linkage = new Linkage(this);

        // Fire up the motors.
        CURRENT_LEVEL = 0;
        CURRENT_ANGLE = 0;
        linkage.GO_TO_LEVEL(CURRENT_LEVEL, 1.0);
        turret.GO_TO_ANGLE(CURRENT_ANGLE, 1.0);
        boolean BRAKE_CHANGED = false;

        turret.RETRACT_BRAKE();
        waitForStart();

        while(opModeIsActive()){
            if(gamepad2.square && !BRAKE_CHANGED) {turret.BRAKE_TOGGLE(); BRAKE_CHANGED = true;}
            else if(!gamepad2.square) BRAKE_CHANGED = false;

            if(gamepad2.left_bumper) CURRENT_LEVEL -= LINKAGE_THRESHOLD;
            if(gamepad2.right_bumper) CURRENT_LEVEL += LINKAGE_THRESHOLD;

            if(CURRENT_LEVEL > SAFETY_THRESHOLD && gamepad2.left_trigger!=0) CURRENT_ANGLE += ANGLE_THRESHOLD;
            if(CURRENT_LEVEL > SAFETY_THRESHOLD && gamepad2.right_trigger!=0) CURRENT_ANGLE -= ANGLE_THRESHOLD;

            linkage.GO_TO_LEVEL(CURRENT_LEVEL, 0.8);
            turret.GO_TO_ANGLE(CURRENT_ANGLE, 0.8);
            telemetry.update();
        }
    }
}
