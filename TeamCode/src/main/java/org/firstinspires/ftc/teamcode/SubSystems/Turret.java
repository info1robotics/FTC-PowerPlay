package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    public DcMotor turretMotor;
    public Servo BrakeServo1, BrakeServo2;
    private static final double GEAR_RATIO = 2.0;
    private static final double TICKS_PER_REVOLUTION = 537.7;
    private static final double ERROR = 2;
    public static final double ANGLE_THRESHOLD = 1;
    public static double CURRENT_ANGLE = 0;
    public static int CURRENT_TICK = 0;
    public static final double MAX_ANGLE = 540;
    public static final int MAX_TICK = 500;
    public static final int MIN_TICK = -500;
    public static final double MIN_ANGLE = -540;

    public Turret(LinearOpMode opMode){
        BrakeServo1 = opMode.hardwareMap.get(Servo.class, "BrakeServo1");
        BrakeServo2 = opMode.hardwareMap.get(Servo.class, "BrakeServo2");
        turretMotor = opMode.hardwareMap.get(DcMotor.class, "Turret");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void RESET_ENCODER() {turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
    public void SET_MOTORS_RUNMODE(){turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);}
    public void SET_TARGET_POSITION(int ANGLE){turretMotor.setTargetPosition(ANGLE);}
    public void SET_MOTOR_POWER(double pw){turretMotor.setPower(pw);}

    public void GO_TO_ANGLE(double ANGLE, double SPEED){
        if(ANGLE > MAX_ANGLE) ANGLE = MAX_ANGLE;
        if(ANGLE < MIN_ANGLE) ANGLE = MIN_ANGLE;
        SET_TARGET_POSITION((int)((TICKS_PER_REVOLUTION * GEAR_RATIO) / (360 / ANGLE) * ERROR));
        SET_MOTORS_RUNMODE();
        SET_MOTOR_POWER(SPEED);
    }

    public void GO_TO_TICK(int TICK_COUNT, double SPEED){
//        if(TICK_COUNT > MAX_TICK) TICK_COUNT = MAX_TICK;
//        if(TICK_COUNT < MIN_TICK) TICK_COUNT = MIN_TICK;
        SET_TARGET_POSITION(TICK_COUNT);
        SET_MOTORS_RUNMODE();
        SET_MOTOR_POWER(SPEED);
    }

    public void ACTIVATE_BRAKE(){
        BrakeServo1.setPosition(0.5);
        BrakeServo2.setPosition(0.1);
    }
    public void RETRACT_BRAKE() {
        BrakeServo1.setPosition(0.8);
        BrakeServo2.setPosition(0.0);
    }
}