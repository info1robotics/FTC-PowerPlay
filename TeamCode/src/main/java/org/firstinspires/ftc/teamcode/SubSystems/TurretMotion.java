package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class TurretMotion {
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
    private final OpMode opMode;
    public double elapsedTime = 0;
    public long lastTimeUpdatedMs = 0;
    public DcMotorEx turretMotor;
    public Servo brakeServo1, brakeServo2;
    public static PIDCoefficients coeffs = new PIDCoefficients(5,  0, 0);
    public static PIDFController controller;
    public MotionProfile currentMotionProfile = null;

    public TurretMotion(LinearOpMode opMode) {
        brakeServo1 = opMode.hardwareMap.get(Servo.class, "BrakeServo1");
        brakeServo2 = opMode.hardwareMap.get(Servo.class, "BrakeServo2");
        turretMotor = opMode.hardwareMap.get(DcMotorEx.class, "Turret");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setTargetPositionTolerance(100);
        lastTimeUpdatedMs = System.currentTimeMillis();
        this.opMode = opMode;
        controller = new PIDFController(coeffs, 0.05);
        controller.setOutputBounds(0, 1);
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

    public void goToAngle(double ANGLE) {
        currentMotionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(turretMotor.getCurrentPosition(), 0, 0),
                new MotionState((int) ((TICKS_PER_REVOLUTION / GEAR_RATIO) / (360 / ANGLE) * ERROR), 0, 0),
                2048, 2048, 0
        );
        elapsedTime = 0;
        controller.reset();
    }

    public double getCurrentAngle(){
//        (int) ((TICKS_PER_REVOLUTION / GEAR_RATIO) / (360 / ANGLE))
        return (turretMotor.getCurrentPosition() * 360 * GEAR_RATIO) / TICKS_PER_REVOLUTION;
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
        elapsedTime += (System.currentTimeMillis() - lastTimeUpdatedMs) / 1000.0;

        if(elapsedTime < currentMotionProfile.duration()) {
            MotionState targetState = currentMotionProfile.get(elapsedTime);

            controller.setTargetPosition(targetState.getX());
            controller.setTargetVelocity(targetState.getV());
            controller.setTargetAcceleration(targetState.getA());

            opMode.telemetry.addData("elapsedTime", elapsedTime);
            opMode.telemetry.addData("duration", currentMotionProfile.duration());
        }

        turretMotor.setPower(controller.update(turretMotor.getCurrentPosition()));

        lastTimeUpdatedMs = System.currentTimeMillis();
    }

}