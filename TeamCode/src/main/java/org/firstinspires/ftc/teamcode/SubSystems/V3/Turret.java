package org.firstinspires.ftc.teamcode.SubSystems.V3;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.CommonPackage.ThreadedIMU;

public class Turret {
    public static int fieldSize = 192; // for cri, default is 6 * 24
    public static final int poleSpacing = 24;
    private final LinearOpMode opMode;

    public static int[] parseCoordinates(String coordinates) {
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
        this.opMode = opMode;
        turretMotor = opMode.hardwareMap.get(DcMotorEx.class, "TurretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setHeading(double targetAngle, double power) {
        setTargetPosition((int) ((TICKS_PER_REVOLUTION / GEAR_RATIO) / (360 / targetAngle)));

        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        setVelocity(power);
    }

    public void setTargetDistance(double distance){ DISTANCE_THRESHOLD = distance; }
    public void setTargetAngle(double angle){ targetAngle = angle; }
    public void setTurretVelocity(double velocity){ turretVelocity = velocity; }
    public void hardReset(){
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setVelocity(double power) {
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

    public void lockOnJunction(String junction, Pose2d robot, double offset) {
        int[] parsedCoordinates = parseCoordinates(junction);
        double robotX = robot.getX();
        double robotY = robot.getY();
        double turretAngle = -getCurrentAngleHeading(); // Negative because fsr clockwise rotation is negative and counter clockwise is positive
        double targetX = parsedCoordinates[0];
        double targetY = parsedCoordinates[1];
        double robotHeading = -ThreadedIMU.getInstance().getHeading();
        double angle = Math.atan2(targetX - robotX, targetY - robotY) - Math.toRadians(turretAngle) - robotHeading;
        double angleDeg = Math.toDegrees(angle);
        double shortestRelative = angleDeg;
        while (shortestRelative > 180) {
            shortestRelative -= 360;
        }
        while (shortestRelative < -180) {
            shortestRelative += 360;
        }
        opMode.telemetry.addData("Shortest Relative", shortestRelative);
        opMode.telemetry.addData("Relative", angleDeg);
        targetAngle = getCurrentAngleHeading() - shortestRelative + offset;
    }

    public void setTargetPosition(int ticks) {
        turretMotor.setTargetPosition(ticks);
    }

    public void debug() {
        opMode.telemetry.addData("Turret's Current Angle Heading ", getCurrentAngleHeading());
        opMode.telemetry.addData("Turret's Current Tick Count ", turretMotor.getCurrentPosition());
    }

    public boolean hasReachedTarget() {
        return Math.abs(turretMotor.getCurrentPosition() - turretMotor.getTargetPosition()) < 10;
    }

    public void update() {
        setHeading(targetAngle, turretVelocity);
    }
}
