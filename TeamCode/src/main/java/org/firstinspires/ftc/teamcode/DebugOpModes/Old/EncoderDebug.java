//package org.firstinspires.ftc.teamcode.DebugOpModes.Old;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//@Disabled
//@TeleOp
//public class EncoderDebug extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//    DcMotor encoder;
//    encoder = hardwareMap.get(DcMotor.class, "encoder");
//    encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    waitForStart();
//    while(opModeIsActive()){
//        telemetry.addData("Encoder Raw Value ", encoder.getCurrentPosition());
//        telemetry.update();
//        }
//    }
//}
