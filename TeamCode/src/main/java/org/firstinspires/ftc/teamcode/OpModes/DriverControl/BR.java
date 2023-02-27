package org.firstinspires.ftc.teamcode.OpModes.DriverControl;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class BR extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "FL");
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "BL_Parallel");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "BR");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "FR_Perpendicular");
        waitForStart();
        rightRear.setPower(1.0);
    }
}
