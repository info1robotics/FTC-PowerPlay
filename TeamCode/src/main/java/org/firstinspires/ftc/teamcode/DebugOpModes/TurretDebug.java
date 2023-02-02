package org.firstinspires.ftc.teamcode.DebugOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Turret;

//@Disabled
@TeleOp
public class TurretDebug extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Turret turret = new Turret(this);
        turret.disengageBrake();
        turret.disengageSuperBrake();
        waitForStart();
        while(opModeIsActive()){
            // Get feedback from the turret motor's encoder for debugging.
            telemetry.addData("turret pos: ", turret.turretMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
