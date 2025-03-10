package org.firstinspires.ftc.teamcode.DebugOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.V2.Linkage;
import org.firstinspires.ftc.teamcode.SubSystems.V3.Turret;

//@Disabled
@TeleOp(name="Upper Systems Debug")
public class UpperSubSystemsDebug extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Linkage linkage = new Linkage(this);
        Turret turret = new Turret(this);
//        Claw claw = new Claw(this);

//        claw.setSubsystemState(Claw.subsystemStates.RETRACTED);
//claw.setSubsystemState(Claw.subsystemStates.READY_TELEOP);
    telemetry.addLine("Warning! This OpMode resets all motor encoders after start is pressed!");
        telemetry.addLine("Claw is open during INIT and closed after start.");
        telemetry.update();
        waitForStart();

        telemetry.clear();
        linkage.resetEncoders();
        turret.resetEncoder();

        while(opModeIsActive()){
//            claw.setPivotPosition(Claw.pivotPositions.INTERMEDIARY);
            telemetry.addData("Left Motor Tick Count ", linkage.linkageLeft.getCurrentPosition());
            telemetry.addData("Right Motor Tick Count ", linkage.linkageRight.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Turret's Current Angle Heading ", turret.getCurrentAngleHeading());
            telemetry.addData("Turret's Current Tick Count ", turret.turretMotor.getCurrentPosition());
            telemetry.addLine();
            telemetry.update();
        }
    }
}
