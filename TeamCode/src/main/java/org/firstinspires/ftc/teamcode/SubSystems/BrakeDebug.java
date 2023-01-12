package org.firstinspires.ftc.teamcode.SubSystems;

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

@TeleOp(name = "Brake Test")
public class BrakeDebug extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize all systems.
        Turret turret = new Turret(this);

        // Fire up the motors.
//        CURRENT_ANGLE = 0;
//        turret.GO_TO_ANGLE(CURRENT_ANGLE, 1.0);
        turret.ACTIVATE_BRAKE();

        waitForStart();
        while(opModeIsActive()){
            // Manually tune the turret angle. The turret will not spin if it is currently at risk of damaging the linkage.
//            if(gamepad2.left_trigger!=0) CURRENT_ANGLE += ANGLE_THRESHOLD;
//            if(gamepad2.right_trigger!=0) CURRENT_ANGLE -= ANGLE_THRESHOLD;

//            turret.GO_TO_ANGLE(CURRENT_ANGLE, 1.0);

            // viteza mica cand e apasat si dupa 1.0 cand nu mai e

            telemetry.addData("Turret Angle ",CURRENT_ANGLE*2);
            telemetry.update();
        }
    }
}
