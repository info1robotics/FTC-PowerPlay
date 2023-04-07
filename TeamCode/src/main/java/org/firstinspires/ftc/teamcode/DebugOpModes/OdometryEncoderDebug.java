package org.firstinspires.ftc.teamcode.DebugOpModes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.V2.Odometry;
@Disabled
@TeleOp(name="Odometry Debug")
public class OdometryEncoderDebug extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Odometry odometry = new Odometry(this);

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Parallel Raw Value ", odometry.rawDistanceParallel());
            telemetry.addData("Perpendicular Raw Value ", odometry.rawDistancePerpendicular());
            telemetry.addLine();
            telemetry.addData("Parallel Distance Traveled ", odometry.translatedDistanceParallel());
            telemetry.addData("Perpendicular Distance Traveled ", odometry.translatedDistancePerpendicular());

            telemetry.update();
        }
    }
}
