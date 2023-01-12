package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.teamcode.SubSystems.Turret.CURRENT_ANGLE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Brake Test")
public class BrakeDebug extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize all systems.
        Turret turret = new Turret(this);
        // Fire up the motors.
//        CURRENT_ANGLE = 0;
//        turret.GO_TO_ANGLE(CURRENT_ANGLE, 1.0);
        turret.RETRACT_BRAKE();

        waitForStart();

        while(opModeIsActive()){
            turret.ACTIVATE_BRAKE();
            telemetry.addData("Brake 1 Position ", turret.BrakeServo1.getPosition());
            telemetry.addData("Brake 2 Position ", turret.BrakeServo2.getPosition());
            telemetry.addData("Turret Angle ",CURRENT_ANGLE*2);
            telemetry.update();
        }
    }
}
