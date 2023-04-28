package org.firstinspires.ftc.teamcode.DebugOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SubSystems.V2.Claw;

//@Disabled
@TeleOp(name="Flip Collect")
public class collectGround extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Claw claw = new Claw(this);
        claw.setPivotPosition(Claw.pivotPositions.DOWNWARDS);
        claw.setClawState(Claw.clawStates.OPEN);
        waitForStart();

        while(opModeIsActive()){
            telemetry.update();
            claw.setClawState(Claw.clawStates.CLOSED);
        }

    }
}
