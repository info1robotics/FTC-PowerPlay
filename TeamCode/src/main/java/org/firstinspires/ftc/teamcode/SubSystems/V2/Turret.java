package org.firstinspires.ftc.teamcode.SubSystems.V2;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Turret {
    public static final double THRESHOLD = 1f;
    private static final double GEAR_RATIO = 36.0 / 231.0;
    private static final double TICKS_PER_REVOLUTION = 384.5;
    public DcMotorEx turretMotor;
    public double LOWER_ANGLE_LIMIT = -360;
    public double HIGHER_ANGLE_LIMIT = 360;

    public Turret(LinearOpMode opMode) {
        turretMotor = opMode.hardwareMap.get(DcMotorEx.class, "TurretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setHeading(double targetAngle, double power) {
        if (targetAngle > HIGHER_ANGLE_LIMIT) targetAngle = HIGHER_ANGLE_LIMIT;
        if (targetAngle < LOWER_ANGLE_LIMIT) targetAngle = LOWER_ANGLE_LIMIT;

        setTargetPosition((int) ((TICKS_PER_REVOLUTION / GEAR_RATIO) / (360 / targetAngle)));
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(power);
    }
    public void hardReset(){
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setPower(double power) {
        turretMotor.setPower(power);
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        turretMotor.setMode(runMode);
    }

    public void resetEncoder() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getCurrentAngleHeading() {
        return (turretMotor.getCurrentPosition() * 360 * GEAR_RATIO) / TICKS_PER_REVOLUTION;
    }

    public void setTargetPosition(int ticks) {
        turretMotor.setTargetPosition(ticks);
    }

    public void debug() {
        telemetry.addData("Turret's Current Angle Heading ", getCurrentAngleHeading());
        telemetry.addData("Turret's Current Tick Count ", turretMotor.getCurrentPosition());
    }

    public boolean hasReachedTarget() {
        return Math.abs(turretMotor.getCurrentPosition() - turretMotor.getTargetPosition()) < 10;
    }
}
