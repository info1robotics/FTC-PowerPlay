package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class Turret {
    public static final double ANGLE_THRESHOLD = 1f;
    public static final double MAX_ANGLE = 540;
    public static final int MAX_TICK = 16000;
    public static final int MIN_TICK = -16000;
    public static final double MIN_ANGLE = -540;
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_REVOLUTION = 8192;
    private static final double ERROR = 1;
    public static boolean brakeState;
    public static double CURRENT_ANGLE = 0;
    public static int CURRENT_TICK = 0;
    public DcMotorEx turretMotor;
    public Servo brakeServo1, brakeServo2;
    public static PIDFCoefficients coeffs = new PIDFCoefficients(0,  0, 0, 0);


    public Turret(LinearOpMode opMode) {
        brakeServo1 = opMode.hardwareMap.get(Servo.class, "BrakeServo1");
        brakeServo2 = opMode.hardwareMap.get(Servo.class, "BrakeServo2");
        turretMotor = opMode.hardwareMap.get(DcMotorEx.class, "Turret");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
//        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setTargetPositionTolerance(200);
    }

    public void setMotorsRunMode(DcMotor.RunMode runMode) {
        turretMotor.setMode(runMode);
    }

    public void setTargetPosition(int tickCount) {
        turretMotor.setTargetPosition(tickCount);
    }

    public void setPower(double pw) {
        turretMotor.setPower(pw);
    }

    public double getCurrentAngle(){
//        (int) ((TICKS_PER_REVOLUTION / GEAR_RATIO) / (360 / ANGLE))
        return (turretMotor.getCurrentPosition() * 360 * GEAR_RATIO) / TICKS_PER_REVOLUTION;
    }

    public void goToAngle(double ANGLE, double SPEED) {
        if (ANGLE > MAX_ANGLE) ANGLE = MAX_ANGLE;
        if (ANGLE < MIN_ANGLE) ANGLE = MIN_ANGLE;
        setTargetPosition((int) ((TICKS_PER_REVOLUTION / GEAR_RATIO) / (360 / ANGLE) * ERROR));
        setMotorsRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(SPEED * ((Math.abs(turretMotor.getCurrentPosition() - turretMotor.getTargetPosition())) / 8192) );

//        if (Math.abs(turretMotor.getCurrentPosition() - turretMotor.getTargetPosition()) < 200) {
////            setPower(0);
////            engageBrake();
////            engageSuperBrake();
////            disengageBrake();
////            disengageSuperBrake();
//        } else {
//            setPower(SPEED);
//            disengageBrake();
//            disengageSuperBrake();
//        }
    }

    public void goToAngleAuto(double ANGLE, double SPEED) {
        if (ANGLE > MAX_ANGLE) ANGLE = MAX_ANGLE;
        if (ANGLE < MIN_ANGLE) ANGLE = MIN_ANGLE;
        setTargetPosition((int) ((TICKS_PER_REVOLUTION / GEAR_RATIO) / (360 / ANGLE) * ERROR));
        setMotorsRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(SPEED);
    }

    public void goToTick(int TICK_COUNT, double SPEED) {
        if (TICK_COUNT > MAX_TICK) TICK_COUNT = MAX_TICK;
        if (TICK_COUNT < MIN_TICK) TICK_COUNT = MIN_TICK;
        setTargetPosition(TICK_COUNT);
        setMotorsRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(SPEED);
    }

    public void toggleBrake() {
        brakeState = !brakeState;
        if (brakeState) {
            brakeServo2.setPosition(0.25);
        } else {
            brakeServo2.setPosition(0.6);
        }
    }

    public void engageBrake() {
        brakeServo2.setPosition(0.25);
    }

    public void disengageBrake() {
        brakeServo2.setPosition(0.6);
    }

    public void engageSuperBrake() {
        brakeServo1.setPosition(0.05);
    }

    public void disengageSuperBrake() {
        brakeServo1.setPosition(0.35);
    }

    public void update() {
        turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, coeffs);
    }

}