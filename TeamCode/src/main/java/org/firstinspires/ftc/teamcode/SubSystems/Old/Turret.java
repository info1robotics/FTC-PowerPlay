//package org.firstinspires.ftc.teamcode.SubSystems;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//@Config
//public class Turret {
//    public static final int MIN_TICK = -16000, MAX_TICK = 16000, MIN_ANGLE = -220, MAX_ANGLE = 220;
//    private static final double GEAR_RATIO = 2.0;
//    private static final double ERROR = 1;
//    private static final double TICKS_PER_REVOLUTION = 3895.9;
//    public static final double ANGLE_THRESHOLD = 1f;
//    public static double CURRENT_ANGLE = 0, DESIRED_ANGLE = 0, CORRECTED_ANGLE = 0, LAST_ANGLE = 0, REVERT_THRESHOLD = 0, AUTO_SPEED = 0.0;
//    public double MIN_DIST, MAX_DIST = 999;
//    public boolean TOGGLE_DIST = false, hardLock = false, correctionFound = false, brakeState, softBrake = false;
//    public Servo brakeServo1, brakeServo2;
//    public DistanceSensor distanceSensor;
//    public DcMotorEx turretMotor;
//    public Turret(LinearOpMode opMode) {
//        distanceSensor = opMode.hardwareMap.get(DistanceSensor.class, "DistanceSensor");
//        brakeServo1 = opMode.hardwareMap.get(Servo.class, "BrakeServo1");
//        brakeServo2 = opMode.hardwareMap.get(Servo.class, "BrakeServo2");
//        turretMotor = opMode.hardwareMap.get(DcMotorEx.class, "Turret");
//        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    public void resetEncoder(){
//        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
//    public void setMotorsRunMode(DcMotor.RunMode runMode) {
//        turretMotor.setMode(runMode);
//    }
//
//    public void setTargetPosition(int tickCount) {
//        turretMotor.setTargetPosition(tickCount);
//    }
//
//    public void setPower(double pw) {
//        turretMotor.setPower(pw);
//    }
//
//    public double getCurrentAngle(){
//        return (turretMotor.getCurrentPosition() * 360 * GEAR_RATIO) / TICKS_PER_REVOLUTION;
//    }
//
//    public void goToAngle(double ANGLE, double SPEED) {
//        if (ANGLE > MAX_ANGLE) ANGLE = MAX_ANGLE;
//        if (ANGLE < MIN_ANGLE) ANGLE = MIN_ANGLE;
//        setTargetPosition((int) ((TICKS_PER_REVOLUTION / GEAR_RATIO) / (360 / ANGLE) * ERROR));
//        setMotorsRunMode(DcMotor.RunMode.RUN_TO_POSITION);
//        if(hardLock){
//            setPower(0);
//            engageBrake();
//            engageSuperBrake();
//        }else if (Math.abs(turretMotor.getCurrentPosition() - turretMotor.getTargetPosition()) < 10) {
//            setPower(0);
//            engageBrake();
//            engageSuperBrake();
//        } else {
//            setPower(SPEED);
//            disengageBrake();
//            disengageSuperBrake();
//        }
//    }
//    public void toggleSetThreshold(boolean bool){
//        TOGGLE_DIST = bool;
//    }
//    public void setDistanceThreshold(double dist){
//        MAX_DIST = dist;
//    }
//
//    public void goToAngleAuto(double SPEED) {
//
//        setTargetPosition((int) ((TICKS_PER_REVOLUTION / GEAR_RATIO) / (360 / CORRECTED_ANGLE) * ERROR));
//        setMotorsRunMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        if(TOGGLE_DIST && distanceSensor.getDistance(DistanceUnit.MM) <= MAX_DIST && !correctionFound){
//            correctionFound = true;
//            CORRECTED_ANGLE = getCurrentAngle() + REVERT_THRESHOLD;
//        }
//        if (Math.abs(turretMotor.getCurrentPosition() - turretMotor.getTargetPosition()) < 20) {
//            setPower(0);
//            engageBrake();
//            engageSuperBrake();
//        } else {
//            setPower(SPEED);
//            disengageBrake();
//            disengageSuperBrake();
//        }
//    }
//    public void goToTick(int TICK_COUNT, double SPEED) {
//        if (TICK_COUNT > MAX_TICK) TICK_COUNT = MAX_TICK;
//        if (TICK_COUNT < MIN_TICK) TICK_COUNT = MIN_TICK;
//        setTargetPosition(TICK_COUNT);
//        setMotorsRunMode(DcMotor.RunMode.RUN_TO_POSITION);
//        setPower(SPEED);
//    }
//
//    public void toggleBrake() {
//        brakeState = !brakeState;
//        if (brakeState) {
//            brakeServo2.setPosition(0.25);
//        } else {
//            brakeServo2.setPosition(0.6);
//        }
//    }
//
//    public void setSoftBrake(boolean state) {
//        softBrake = state;
//    }
//    public void engageBrake() {
//        brakeServo2.setPosition(0.35);
//    }
//
//    public void disengageBrake() {
//        brakeServo2.setPosition(0.6);
//    }
//
//    public void engageSuperBrake() {
//        brakeServo1.setPosition(0.30);
//    }
//
//    public void disengageSuperBrake() {
//        brakeServo1.setPosition(0.55);
//    }
//
//    public void updateAuto() {
//       if (LAST_ANGLE != DESIRED_ANGLE) {CORRECTED_ANGLE = DESIRED_ANGLE;
//       correctionFound = false;}
//       LAST_ANGLE = DESIRED_ANGLE;
//       goToAngleAuto(AUTO_SPEED);
//    }
//
//}