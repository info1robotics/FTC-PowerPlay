package org.firstinspires.ftc.teamcode.SubSystems.V2;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
public class Odometry {
    DcMotorEx odoLeft, odoRight, odoCenter;
    private static final double WHEEL_DIAMETER = 2;
    private static final double WHEEL_DIAMETER_TO_NORMAL_UNITS = WHEEL_DIAMETER * 2.54;
    static final double WHEEL_RADIUS = WHEEL_DIAMETER_TO_NORMAL_UNITS / 2;
    static final double WHEEL_CIRCUMFERENCE = Math.PI * 2 * WHEEL_RADIUS;
    static final double TICKS_PER_REVOLUTION = 8192;
    static double HEADING = 0;
    static final double WHEELBASE_WIDTH = (10.6 + 16.1) / 2;
    static final double CENTER_WHEEL_LENGTH = (8.7 + 5.9) / 2;

    static final double LATERAL_ANGULAR_OFFSET = ((5 * 90) / ( 86.75 + 88.06 + 87.44 + 87.14 + 84.9 ));
    static final double PERPENDICULAR_ANGULAR_OFFSET = ((5 * 90) / ( 112.23 + 123.37 + 124.90 + 111.19 + 115.67 ));

    public Odometry(LinearOpMode opMode) {
        odoLeft = opMode.hardwareMap.get(DcMotorEx.class, "FL_Left");
        odoRight = opMode.hardwareMap.get(DcMotorEx.class, "FR_Right");
        odoCenter = opMode.hardwareMap.get(DcMotorEx.class, "BR_Center");

        resetEncoders();

        odoRight.setDirection(DcMotorSimple.Direction.REVERSE);
        odoCenter.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void resetEncoders() {
        odoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setHeading(double angle){ HEADING = angle; }
    public double rawDistanceLeft(){ return odoLeft.getCurrentPosition(); }
    public double rawDistanceRight(){ return odoRight.getCurrentPosition(); }
    public double rawDistanceCenter(){ return odoCenter.getCurrentPosition(); }
    public double translatedDistanceLeft(){ return (odoLeft.getCurrentPosition() / TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE; }
    public double translatedDistanceRight(){ return (odoRight.getCurrentPosition() / TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE; }
    public double translatedDistanceCenter(){ return (odoCenter.getCurrentPosition() / TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE; }
    public double getCurrentHeading(){ return Math.toDegrees(((translatedDistanceRight() - translatedDistanceLeft()) / WHEELBASE_WIDTH) * LATERAL_ANGULAR_OFFSET); }
    public void debug(){
        telemetry.addData("Left Raw Value ", rawDistanceLeft());
        telemetry.addData("Right Raw Value ", rawDistanceRight());
        telemetry.addData("Center Raw Value ", rawDistanceCenter());
        telemetry.addLine();
        telemetry.addData("Left Distance Traveled ", translatedDistanceLeft());
        telemetry.addData("Right Distance Traveled ", translatedDistanceRight());
        telemetry.addData("Center Distance Traveled ", translatedDistanceCenter());
    }
}

