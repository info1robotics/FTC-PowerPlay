package org.firstinspires.ftc.teamcode.DebugOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.SubSystems.TurretMotion;

//@Disabled
@TeleOp
public class TurretDebug extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TurretMotion turret = new TurretMotion(this);
        turret.disengageBrake();
        turret.disengageSuperBrake();



        waitForStart();

        turret.goto0();
        while(opModeIsActive()){

            turret.update();
            telemetry.addData("turret angle: ", turret.getCurrentAngle());
            telemetry.update();
        }
    }
}
