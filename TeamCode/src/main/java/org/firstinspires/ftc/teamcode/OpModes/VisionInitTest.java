package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.EOCV.AprilTagAutonomousInitDetectionExample;

@Autonomous(name="initshit")
public class VisionInitTest extends StateOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("detected:", AprilTagAutonomousInitDetectionExample.zone);
        telemetry.update();
        waitForStart();
    }
}
