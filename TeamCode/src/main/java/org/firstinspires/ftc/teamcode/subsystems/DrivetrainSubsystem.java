package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.moonshine.Subsystem;
import org.firstinspires.ftc.teamcode.moonshine.annotations.RequireHardware;

public class DrivetrainSubsystem extends Subsystem {

    double x, y, turn;

    @RequireHardware(hardwareName = "motorFL") DcMotorEx motorFL;
    @RequireHardware(hardwareName = "motorFR") DcMotorEx motorFR;
    @RequireHardware(hardwareName = "motorBL") DcMotorEx motorBL;
    @RequireHardware(hardwareName = "motorBR") DcMotorEx motorBR;

    public void setMoveVector(double x, double y, double turn) {
        this.x = x;
        this.y = y;
        this.turn = turn;
    }

    @Override
    protected void onStart() {

    }

    @Override
    protected void onTick() {
        motorFL.setPower();
    }

    @Override
    protected void onEnd() {

    }

}
