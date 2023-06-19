package org.firstinspires.ftc.teamcode.SubSystems.V3;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.StandardTrackingWheelLocalizer;

public class Turret {
    public static final int fieldSize = 192;
    public static final int poleSpacing = 24;

    private static int[] parseCoordinates(String coordinates) {
        int[] parsed = new int[2];

        char letter = coordinates.charAt(0);
        int number = Integer.parseInt(coordinates.substring(1));

        parsed[0] = parseLetterToX(letter);
        parsed[1] = parseNumberToY(number);

        return parsed;
    }

    private static int parseLetterToX(char letter) {
        int poleOffset = poleSpacing * (letter - 'A');
        return -fieldSize / 2 + poleOffset + 24;
    }

    private static int parseNumberToY(int number) {
        int poleOffset = poleSpacing * (number - 1);
        return -fieldSize / 2 + poleOffset + 24;
    }

    private static final double GEAR_RATIO = 36.0 / 231.0;
    private static final double TICKS_PER_REVOLUTION = 384.5; // 435 RPM
    public DcMotorEx turretMotor;
    public double DISTANCE_THRESHOLD = 0.0;
    public static double targetAngle = 0.0, turretVelocity = 1.0;

    public Turret(LinearOpMode opMode) {
        turretMotor = opMode.hardwareMap.get(DcMotorEx.class, "TurretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setHeading(double targetAngle, double power) {
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

    public void lockOnJunction(String junction) {
        int[] parsedCoordinates = parseCoordinates(junction);
        double robotX = StandardTrackingWheelLocalizer.instance.getWheelPositions().get(0);
        double robotY = StandardTrackingWheelLocalizer.instance.getWheelPositions().get(1);
        double turretAngle = getCurrentAngleHeading();
        double targetX = parsedCoordinates[0];
        double targetY = parsedCoordinates[1];
//        System.out.println("Target X: " + targetX);
//        System.out.println("Target Y: " + targetY);
        double angle = Math.atan2(targetX - robotX, targetY - robotY) - Math.toRadians(turretAngle);
//        System.out.println("Target Angle: " + Math.toDegrees(angle));
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
