package org.firstinspires.ftc.teamcode.DebugOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SubSystems.V2.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.V2.Odometry;
@Disabled
@TeleOp(name="LoonyClaw Debug")
public class LoonyClawDebug extends LinearOpMode {
    Servo loonyServo;
    @Override
    public void runOpMode() throws InterruptedException {
        Claw claw = new Claw(this);
        claw.setClawState(Claw.clawStates.OPEN);
        waitForStart();

        while(opModeIsActive()){
            telemetry.update();
        }
    }
}
