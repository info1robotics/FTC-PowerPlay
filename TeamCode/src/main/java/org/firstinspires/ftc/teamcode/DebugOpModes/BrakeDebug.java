package org.firstinspires.ftc.teamcode.DebugOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Turret;

//@Disabled
@TeleOp(name = "BrakeDebug")
public class BrakeDebug extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Turret turret = new Turret(this);
        turret.disengageBrake();
        turret.disengageSuperBrake();

        waitForStart();
        while(opModeIsActive()){
            turret.engageBrake();
            turret.engageSuperBrake();
            telemetry.update();
        }
    }
}
