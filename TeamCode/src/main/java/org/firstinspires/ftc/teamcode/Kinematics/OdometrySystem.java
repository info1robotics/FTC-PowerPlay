package org.firstinspires.ftc.teamcode.Kinematics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OdometrySystem {

    DcMotor odoLeft, odoRight, odoCenter;
    private static final double WHEEL_DIAMETER = 2;
    private static final double WHEEL_DIAMETER_TO_NORMAL_UNITS = WHEEL_DIAMETER * 2.54;
    static final double WHEEL_RADIUS = WHEEL_DIAMETER_TO_NORMAL_UNITS / 2;
    static final double WHEEL_CIRCUMFERENCE = Math.PI * 2 * WHEEL_RADIUS;
    static final double TICKS_PER_REVOLUTION = 8192;
    static double LEFT_ODOMETRY_DISTANCE_TRAVELED;
    static double RIGHT_ODOMETRY_DISTANCE_TRAVELED;
    static double CENTER_ODOMETRY_DISTANCE_TRAVELED;
    static double HEADING = 0;

    public OdometrySystem(HardwareMap hardwareMap) {
        odoLeft = hardwareMap.get(DcMotor.class, "OdometryLeft");
        odoRight = hardwareMap.get(DcMotor.class, "OdometryRight");
        odoCenter = hardwareMap.get(DcMotor.class, "OdometryBack");

        odoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        odoRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void manualReset() {
        odoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setHeading(double angle){HEADING = angle;}

    public double rawDistanceLeft(){return odoLeft.getCurrentPosition();}
    public double rawDistanceRight(){return odoRight.getCurrentPosition();}
    public double rawDistanceCenter(){return odoCenter.getCurrentPosition();}

    public double translatedDistanceLeft(){
        LEFT_ODOMETRY_DISTANCE_TRAVELED = (odoLeft.getCurrentPosition() / TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE;
        return LEFT_ODOMETRY_DISTANCE_TRAVELED;
    }
    public double translatedDistanceRight(){
        RIGHT_ODOMETRY_DISTANCE_TRAVELED = (odoRight.getCurrentPosition() / TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE;
        return RIGHT_ODOMETRY_DISTANCE_TRAVELED;
    }
    public double translatedDistanceCenter(){
        CENTER_ODOMETRY_DISTANCE_TRAVELED = (odoCenter.getCurrentPosition() / TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE;
        return CENTER_ODOMETRY_DISTANCE_TRAVELED;
    }

    public double getCurrentLateralHeading(){
        double lateral_alpha = (rawDistanceLeft() + Math.abs(rawDistanceRight())) / (133.5);
        return lateral_alpha;
    }

    public double getCurrentBackHeading(){
        double back_alpha = rawDistanceCenter();
        return back_alpha;
    }

    public double getCurrentComputedHeading(){
        double alpha = (getCurrentBackHeading() + getCurrentLateralHeading()) / 2;
        return alpha;
    }

    public double computeHeading(){
        double finalAngle = 0;
        return finalAngle;
    }

}

