package org.firstinspires.ftc.teamcode.DebugOpModes;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

//@Disabled
@TeleOp(name = "DistanceDebug")
public class DistanceSensorDebug extends LinearOpMode {
    DistanceSensor distance;

    @Override
    public void runOpMode() throws InterruptedException {

        distance = hardwareMap.get(DistanceSensor.class, "DistanceSensor");
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Current Distance in Freedom Units ", distance.getDistance(DistanceUnit.INCH));
            telemetry.addData("Current Distance in Slave Units ", distance.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
