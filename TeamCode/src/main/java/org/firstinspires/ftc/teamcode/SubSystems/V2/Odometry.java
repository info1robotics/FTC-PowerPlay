package org.firstinspires.ftc.teamcode.SubSystems.V2;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
public class Odometry {
    DcMotorEx odoParallel, odoPerpendicular;
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
        odoParallel = opMode.hardwareMap.get(DcMotorEx.class, "BL_Parallel");
        odoPerpendicular = opMode.hardwareMap.get(DcMotorEx.class, "FR_Perpendicular");

        resetEncoders();

        odoParallel.setDirection(DcMotorSimple.Direction.REVERSE);
        odoPerpendicular.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void resetEncoders() {
        odoParallel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoPerpendicular.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setHeading(double angle){ HEADING = angle; }
    public double rawDistanceParallel(){ return odoParallel.getCurrentPosition(); }
    public double rawDistancePerpendicular(){ return odoPerpendicular.getCurrentPosition(); }
    public double translatedDistanceParallel(){ return (odoParallel.getCurrentPosition() / TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE; }
    public double translatedDistancePerpendicular(){ return (odoPerpendicular.getCurrentPosition() / TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE; }
    public void debug(){
        telemetry.addData("Parallel Raw Value ", rawDistanceParallel());
        telemetry.addData("Perpendicular Raw Value ", rawDistancePerpendicular());
        telemetry.addLine();
        telemetry.addData("Parallel Distance Traveled ", translatedDistanceParallel());
        telemetry.addData("Perpendicular Distance Traveled ", translatedDistancePerpendicular());
    }
}

