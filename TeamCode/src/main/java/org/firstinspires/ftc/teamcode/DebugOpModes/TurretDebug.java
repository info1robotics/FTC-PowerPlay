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
        long initial = System.nanoTime();
        long estimated;
        Turret turret = new Turret(this);
        turret.disengageBrake();
        turret.disengageSuperBrake();
        waitForStart();

//        turret.goto0();
        while(opModeIsActive()){
            estimated = System.nanoTime() - initial;
            telemetry.addData("timestamp: ", estimated);
            telemetry.addData("turret angle: ", turret.getCurrentAngle());
//            telemetry.addData("encoder angle: ", turret.getCurrentEncoderAngle());
            telemetry.update();
        }
    }
}
