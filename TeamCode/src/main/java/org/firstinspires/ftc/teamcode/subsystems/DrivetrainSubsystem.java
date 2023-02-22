package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.PPController;
import org.firstinspires.ftc.teamcode.moonshine.Subsystem;
import org.firstinspires.ftc.teamcode.moonshine.annotations.RequireHardware;

public class DrivetrainSubsystem extends Subsystem {

    double x, y, turn, speed;
    PPController controller;

    @RequireHardware(hardwareName = "motorFL") DcMotorEx motorFL;
    @RequireHardware(hardwareName = "motorFR") DcMotorEx motorFR;
    @RequireHardware(hardwareName = "motorBL") DcMotorEx motorBL;
    @RequireHardware(hardwareName = "motorBR") DcMotorEx motorBR;

    public void setMoveVector(double x, double y, double turn, double speed) {
        this.x = x;
        this.y = y;
        this.turn = turn;
        this.speed = speed;
    }

    @Override
    protected void onStart() {
        setMoveVector(0, 0, 0, 0);
        controller = new PPController();
    }

    @Override
    protected void onTick() {
        double[] powers = getMotorPowersArray();
        motorFL.setPower(powers[0]);
        motorBL.setPower(powers[1]);
        motorFR.setPower(powers[2]);
        motorBR.setPower(powers[3]);
    }

    double[] getMotorPowersArray() {
        double[] powers = new double[] {
            y + x + turn,
            y - x + turn,
            y - x - turn,
            y + x - turn
        };

        double max = 0;
        for(double d : powers) {
            max = Math.max(d, max);
        }
        for(int i = 0; i < powers.length; i++) {
            powers[i] *= 1 / max * speed;
        }

        return powers;
    }

    @Override
    protected void onEnd() {

    }

}
