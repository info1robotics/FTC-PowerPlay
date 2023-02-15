package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptRevSPARKMini;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Roadrunner.util.Encoder;

@Config
public class Turret {
    public static final double ANGLE_THRESHOLD = 1f;
    public static double MAX_ANGLE = 220;
    public static final int MAX_TICK = 16000;
    public static final int MIN_TICK = -16000;
    public static double MIN_ANGLE = -220;
    private static final double GEAR_RATIO = 2.0;
    private static final double TICKS_PER_REVOLUTION = 3895.9;

    private static final double ENCODER_TICKS_PER_REVOLUTION = 8192;
    private static final double ERROR = 1;
    public static boolean brakeState;
    public static double CURRENT_ANGLE = 0;
    public static int CURRENT_TICK = 0;
    public static double DESIRED_ANGLE = 0, CORRECTED_ANGLE = 0, LAST_ANGLE = 0;
    public DcMotorEx turretMotor;
    public Servo brakeServo1, brakeServo2;
    public DcMotor turretEncoder;
    public DistanceSensor distanceSensor;
    public double MIN_DIST, MAX_DIST = 999;
    public boolean TOGGLE_DIST = false;
    public boolean hardLock = false;
    public boolean correctionFound = false;
    public static double CORRECTED_THRESHOLD = 0;
    public static double REVERT_THRESHOLD = 0;
    public static double AUTO_SPEED = 0.0;

    public boolean softBrake = false;

    public Turret(LinearOpMode opMode) {
        distanceSensor = opMode.hardwareMap.get(DistanceSensor.class, "DistanceSensor");
        turretEncoder = opMode.hardwareMap.get(DcMotor.class, "TurretEncoder");
        brakeServo1 = opMode.hardwareMap.get(Servo.class, "BrakeServo1");
        brakeServo2 = opMode.hardwareMap.get(Servo.class, "BrakeServo2");
        turretMotor = opMode.hardwareMap.get(DcMotorEx.class, "Turret");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetEncoder(){
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        return (turretMotor.getCurrentPosition() * 360 * GEAR_RATIO) / TICKS_PER_REVOLUTION;
    }

    public double getCurrentEncoderAngle(){
        return (turretEncoder.getCurrentPosition() * 360) / ENCODER_TICKS_PER_REVOLUTION;
    }

    public void goToAngle(double ANGLE, double SPEED) {
        if (ANGLE > MAX_ANGLE) ANGLE = MAX_ANGLE;
        if (ANGLE < MIN_ANGLE) ANGLE = MIN_ANGLE;
        setTargetPosition((int) ((TICKS_PER_REVOLUTION / GEAR_RATIO) / (360 / ANGLE) * ERROR));
        setMotorsRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(hardLock){
            setPower(0);
            engageBrake();
            engageSuperBrake();
        }else if (Math.abs(turretMotor.getCurrentPosition() - turretMotor.getTargetPosition()) < 10) {
            setPower(0);
            engageBrake();
            engageSuperBrake();
        } else {
            setPower(SPEED);
            disengageBrake();
            disengageSuperBrake();
        }
    }

    public void toggleSetThreshold(boolean bool){
        TOGGLE_DIST = bool;
    }
    public void setDistanceThreshold(double dist){
        MAX_DIST = dist;
    }

    public void goToAngleHard(double SPEED) {

        setTargetPosition((int) ((TICKS_PER_REVOLUTION / GEAR_RATIO) / (360 / CORRECTED_ANGLE) * ERROR));
        setMotorsRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(TOGGLE_DIST && distanceSensor.getDistance(DistanceUnit.MM) <= MAX_DIST && !correctionFound){
            correctionFound = true;
            CORRECTED_ANGLE = getCurrentAngle() + REVERT_THRESHOLD;
//            setPower(0);
//            engageBrake();
//            engageSuperBrake();
//            return;
        }
        if (Math.abs(turretMotor.getCurrentPosition() - turretMotor.getTargetPosition()) < 20) {
            setPower(0);
            engageBrake();
            engageSuperBrake();
        } else {
            setPower(SPEED);
            disengageBrake();
            disengageSuperBrake();
        }
    }

//    public void goToAngleAuto(double SPEED) {
////        if(Math.abs(getCurrentAngle()-DESIRED_ANGLE) < 2) CORRECTED_ANGLE = -(DESIRED_ANGLE + (DESIRED_ANGLE - getCurrentEncoderAngle()));
////        if(Math.abs(DESIRED_ANGLE-getCurrentEncoderAngle()) <= 1.0) CORRECTED_ANGLE = getCurrentAngle();
//        if(DESIRED_ANGLE == 0) DESIRED_ANGLE = 0.1;
//
//        if(Math.abs(DESIRED_ANGLE - getCurrentEncoderAngle()) >= 45) CORRECTED_THRESHOLD = 5;
//        if(Math.abs(DESIRED_ANGLE - getCurrentEncoderAngle()) >= 30 && Math.abs(DESIRED_ANGLE - getCurrentEncoderAngle()) < 45) CORRECTED_THRESHOLD = 1.0;
//        if(Math.abs(DESIRED_ANGLE - getCurrentEncoderAngle()) >= 15 && Math.abs(DESIRED_ANGLE - getCurrentEncoderAngle()) < 30) CORRECTED_THRESHOLD = 0.75;
//        if(Math.abs(DESIRED_ANGLE - getCurrentEncoderAngle()) >= 2 && Math.abs(DESIRED_ANGLE - getCurrentEncoderAngle()) < 15) CORRECTED_THRESHOLD = 0.5;
//        if(Math.abs(DESIRED_ANGLE - getCurrentEncoderAngle()) < 2) CORRECTED_THRESHOLD = 0.25;
//
//
//        if (Math.abs(DESIRED_ANGLE - getCurrentEncoderAngle()) > 2){
//            if(getCurrentEncoderAngle() < DESIRED_ANGLE) CORRECTED_ANGLE += ((Math.abs(DESIRED_ANGLE) / DESIRED_ANGLE) * CORRECTED_THRESHOLD);
//            if(getCurrentEncoderAngle() > DESIRED_ANGLE) CORRECTED_ANGLE -= ((Math.abs(DESIRED_ANGLE) / DESIRED_ANGLE) * CORRECTED_THRESHOLD);
//        }
//
//        setTargetPosition((int) ((TICKS_PER_REVOLUTION / GEAR_RATIO) / (360 / CORRECTED_ANGLE) * ERROR));
//        setMotorsRunMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        if (Math.abs(DESIRED_ANGLE - getCurrentEncoderAngle()) <= 1.5) {
//        setPower(0);
//        engageBrake();
//        engageSuperBrake();
//        } else {
//        setPower(SPEED);
//        disengageBrake();
//        disengageSuperBrake();
//    }
//    }

//            public void goToAngleAuto(double SPEED) {
////        if(Math.abs(getCurrentAngle()-DESIRED_ANGLE) < 2) CORRECTED_ANGLE = -(DESIRED_ANGLE + (DESIRED_ANGLE - getCurrentEncoderAngle()));
//                if (Math.abs(DESIRED_ANGLE - getCurrentEncoderAngle()) <= 1.0)
//                    CORRECTED_ANGLE = getCurrentAngle();
//
//                setTargetPosition((int) ((TICKS_PER_REVOLUTION / GEAR_RATIO) / (360 / CORRECTED_ANGLE) * ERROR));
//                setMotorsRunMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                if (Math.abs(DESIRED_ANGLE - getCurrentEncoderAngle()) < 2) {
//                    setPower(0);
//                    engageBrake();
//                    engageSuperBrake();
//                } else {
//                    setPower(SPEED);
//                    disengageBrake();
//                    disengageSuperBrake();
//                }
//            }

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

    public void setSoftBrake(boolean state) {
        softBrake = state;
    }

    public void engageBrake() {
        brakeServo2.setPosition(0.35);
    }

    public void disengageBrake() {
        brakeServo2.setPosition(0.6);
    }

    public void engageSuperBrake() {
        brakeServo1.setPosition(0.1);
    }

    public void disengageSuperBrake() {
        brakeServo1.setPosition(0.35);
    }

    public void update() {
       if (LAST_ANGLE != DESIRED_ANGLE) {CORRECTED_ANGLE = DESIRED_ANGLE;
       correctionFound = false;}
       LAST_ANGLE = DESIRED_ANGLE;
        goToAngleHard(AUTO_SPEED);

        // soft braking
//        if(softBrake) {
//            if(turretMotor.getPower() >= 0) {
//
//                // disengage super servo and engage normal one
//                brakeServo2.setPosition(0.35);
//                brakeServo1.setPosition(0.25);
//            } else {
//                brakeServo1.setPosition(0.05);
//                brakeServo2.setPosition(0.6);
//            }
//        }
    }

}