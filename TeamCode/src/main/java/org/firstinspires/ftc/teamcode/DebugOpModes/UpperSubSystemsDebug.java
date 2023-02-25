package org.firstinspires.ftc.teamcode.DebugOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.V2.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.V2.Linkage;
import org.firstinspires.ftc.teamcode.SubSystems.V2.Turret;

//@Disabled
@TeleOp(name="Upper Systems Debug")
public class UpperSubSystemsDebug extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Linkage linkage = new Linkage(this);
        Turret turret = new Turret(this);
        Claw claw = new Claw(this);

        claw.setState(Claw.states.OPEN);
        telemetry.addLine("Warning! This OpMode resets all motor encoders after start is pressed!");
        telemetry.addLine("Claw is open during INIT and closed after start.");
        telemetry.update();

        waitForStart();

        telemetry.clear();
        linkage.resetEncoders();
        turret.resetEncoder();

        while(opModeIsActive()){
            linkage.debug();
            telemetry.addLine();
            turret.debug();
            telemetry.addLine();
            telemetry.update();
        }
    }
}
