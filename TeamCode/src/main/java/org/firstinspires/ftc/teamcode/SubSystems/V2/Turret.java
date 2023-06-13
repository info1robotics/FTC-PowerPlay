package org.firstinspires.ftc.teamcode.SubSystems.V2;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.StandardTrackingWheelLocalizer;

public class Turret {
    // TODO: Make sure all coordinates are correct
    public enum Junction {
        LOW_BOTTOM_LEFT(24, 12),
        LOW_BOTTOM_RIGHT(48, 12),
        LOW_STACK_LEFT(12, 24),
        LOW_STACK_RIGHT(60, 24),
        MID_LEFT(24, 24),
        MID_RIGHT(48, 24),
        HIGH_CENTER(36, 24),
        HIGH_LEFT(24, 36),
        HIGH_RIGHT(48, 36);

        private final int x;
        private final int y;

        Junction(int x, int y) {
            this.x = x;
            this.y = y;
        }
    }

    public static final double THRESHOLD = 1f;
    private static final double GEAR_RATIO = 36.0 / 231.0;
    private static final double TICKS_PER_REVOLUTION = 290.2;
    public DcMotorEx turretMotor;
    public DistanceSensor distanceSensor;
    public double LOWER_ANGLE_LIMIT = -360;
    public double HIGHER_ANGLE_LIMIT = 360;
    public double DISTANCE_THRESHOLD = 0.0;
    public boolean useSensor = false;
    public static double targetAngle = 0.0, correctedAngle = 0.0, lastAngle = 0.0, turretVelocity = 1.0;

    public Turret(LinearOpMode opMode) {
        turretMotor = opMode.hardwareMap.get(DcMotorEx.class, "TurretMotor");
//        distanceSensor = opMode.hardwareMap.get(DistanceSensor.class, "DistanceSensor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setHeading(double targetAngle, double power) {
//        if (targetAngle > HIGHER_ANGLE_LIMIT) targetAngle = HIGHER_ANGLE_LIMIT;
//        if (targetAngle < LOWER_ANGLE_LIMIT) targetAngle = LOWER_ANGLE_LIMIT;

        setTargetPosition((int) ((TICKS_PER_REVOLUTION / GEAR_RATIO) / (360 / targetAngle)));

        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(power);
    }

    public void setTargetDistance(double distance){ DISTANCE_THRESHOLD = distance; }
    public void setTargetAngle(double angle){ targetAngle = angle; }
    public void setTurretVelocity(double velocity){ turretVelocity = velocity; }
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

    public void lockOnJunction(Junction junction) {
        double robotX = StandardTrackingWheelLocalizer.instance.getWheelPositions().get(0);
        double robotY = StandardTrackingWheelLocalizer.instance.getWheelPositions().get(1);
        double turretAngle = getCurrentAngleHeading();
        double targetX = junction.x;
        double targetY = junction.y;
        double angle = Math.atan2(targetY - robotY, targetX - robotX) - Math.toRadians(turretAngle);
        targetAngle = Math.toDegrees(angle);
        turretVelocity = 1;
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

    public void update(){
        setHeading(targetAngle, turretVelocity);
    }
}
