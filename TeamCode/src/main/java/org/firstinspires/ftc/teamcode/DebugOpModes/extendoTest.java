package org.firstinspires.ftc.teamcode.DebugOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.V2.Odometry;
import org.firstinspires.ftc.teamcode.SubSystems.V3.Extendo;
import org.firstinspires.ftc.teamcode.SubSystems.V3.Lift;

@TeleOp
public class extendoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
//        Extendo extendo = new Extendo(this);
        Lift lift = new Lift(this);

        lift.resetEncoders();
        lift.setHeight(2500, 1.0);

//        extendo.setState(Extendo.ExtendoState.RETRACTED);
        telemetry.addLine("Retracted");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){
            lift.setHeight(0, 1.0);
            telemetry.addData("Left", lift.liftLeft.getCurrentPosition());
            telemetry.addData("Right", lift.liftRight.getCurrentPosition());
//            extendo.setState(Extendo.ExtendoState.FULL);
            telemetry.addLine("Extended");
            telemetry.update();
        }
    }
}
