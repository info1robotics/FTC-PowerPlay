package org.firstinspires.ftc.teamcode.DebugOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.V2.Odometry;

@TeleOp(name="Odometry Debug")
public class OdometryEncoderDebug extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Odometry odometry = new Odometry(this);

        waitForStart();

        while(opModeIsActive()){
            odometry.debug();
            telemetry.update();
        }
    }
}
