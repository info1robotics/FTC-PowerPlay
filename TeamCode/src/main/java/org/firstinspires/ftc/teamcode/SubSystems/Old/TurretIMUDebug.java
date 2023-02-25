//package org.firstinspires.ftc.teamcode.SubSystems;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//@Disabled
//@TeleOp
//public class TurretIMUDebug extends OpMode {
//
//    TurretIMU turret;
//
//    @Override
//    public void init() {
//        turret = new TurretIMU(hardwareMap);
//        turret.setTargetAngle(350);
//
//
//    }
//
//    @Override
//    public void loop() {
//        turret.update();
//        telemetry.addData("current angle", turret.getAngle());
//        telemetry.addData("target angle", turret.getTargetAngle());
//        telemetry.addData("delta angle", Math.abs(turret.getAngle() - turret.getTargetAngle()));
//        telemetry.update();
//    }
//}
