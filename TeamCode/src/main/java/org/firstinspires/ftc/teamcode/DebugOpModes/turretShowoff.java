package org.firstinspires.ftc.teamcode.DebugOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.V2.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.V3.Turret;

//@Disabled
@TeleOp(name="showoff")
public class turretShowoff extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Turret turret = new Turret(this);
        Claw claw = new Claw(this);
        claw.setSubsystemState(Claw.subsystemStates.READY);
        waitForStart();
        telemetry.clear();
        turret.resetEncoder();
        while(opModeIsActive()){
            turret.setTargetAngle(1080);
            turret.setTurretVelocity(0.5);
            turret.update();
            telemetry.update();
        }
    }
}
