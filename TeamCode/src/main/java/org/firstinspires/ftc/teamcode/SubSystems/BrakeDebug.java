package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.teamcode.SubSystems.Linkage.CURRENT_LEVEL;
import static org.firstinspires.ftc.teamcode.SubSystems.Linkage.LINKAGE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.SubSystems.Linkage.SAFETY_THRESHOLD;
import static org.firstinspires.ftc.teamcode.SubSystems.Turret.ANGLE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.SubSystems.Turret.CURRENT_ANGLE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BrakeDebug")
public class BrakeDebug extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize all systems.
        Turret turret = new Turret(this);
        turret.GO_TO_ANGLE(CURRENT_ANGLE, 1.0);
        boolean BRAKE_CHANGED = false;

        turret.RETRACT_BRAKE();
        waitForStart();

        while(opModeIsActive()){
            if(gamepad2.square && !BRAKE_CHANGED) {turret.BRAKE_TOGGLE(); BRAKE_CHANGED = true;}
            else if(!gamepad2.square) BRAKE_CHANGED = false;

            if(gamepad2.left_trigger!=0) CURRENT_ANGLE += ANGLE_THRESHOLD;
            if(gamepad2.right_trigger!=0) CURRENT_ANGLE -= ANGLE_THRESHOLD;

            turret.GO_TO_ANGLE(CURRENT_ANGLE, 0.8);
            telemetry.update();
        }
    }
}
