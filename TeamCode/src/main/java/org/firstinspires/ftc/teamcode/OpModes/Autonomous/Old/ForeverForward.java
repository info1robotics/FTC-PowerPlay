//package org.firstinspires.ftc.teamcode.OpModes.Autonomous;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.SubSystems.V2.Drivetrain;
//
//@Autonomous
//public class ForeverForward extends LinearOpMode {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Drivetrain drivetrain = new Drivetrain(hardwareMap);
//        waitForStart();
//        drivetrain.vectorMove(0.0, 1.0, 0.0, 1.0);
//        while (opModeIsActive() && !isStopRequested());
//    }
//}
