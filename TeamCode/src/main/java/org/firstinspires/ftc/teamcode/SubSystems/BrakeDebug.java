package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.teamcode.SubSystems.Turret.ANGLE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.SubSystems.Turret.CURRENT_ANGLE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp(name = "BrakeDebug")
public class BrakeDebug extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize all systems.
        Turret turret = new Turret(this);
//        turret.goToAngle(CURRENT_ANGLE, 1.0);
        boolean BRAKE_CHANGED = false;
        turret.disengageBrake();
        waitForStart();

        while(opModeIsActive()){
            turret.engageBrake();
            if(gamepad2.square && !BRAKE_CHANGED) {turret.toggleBrake(); BRAKE_CHANGED = true;}
            else if(!gamepad2.square) BRAKE_CHANGED = false;
            telemetry.update();
        }
    }
}
