package org.firstinspires.ftc.teamcode.RoadRunner.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.util.Encoder;

import java.util.Arrays;
import java.util.List;

public class StandardTrackingWheelLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.98; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = -3.346; // X is the up and down direction
    public static double PARALLEL_Y = 1.969; // Y is the strafe direction

    public static double PERPENDICULAR_X = -2.953;
    public static double PERPENDICULAR_Y = -1.378;

    public static double X_MULTIPLIER = 0.9940261261; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1.002019; // Multiplier in the Y direction

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    public final Encoder parallelEncoder;
    public final Encoder perpendicularEncoder;
    private final IMU imu;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        imu = hardwareMap.get(IMU.class, "imu");

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BL_Parallel"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FR_Perpendicular"));
        perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }


    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @Nullable
    @Override
    public Double getHeadingVelocity() {
        return (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        System.out.println("WHEEL VELOCITIES");
        System.out.println((parallelEncoder.getCorrectedVelocity()) * X_MULTIPLIER);
        System.out.println((perpendicularEncoder.getCorrectedVelocity()) * Y_MULTIPLIER);
        System.out.println(encoderTicksToInches((int) parallelEncoder.getCorrectedVelocity()) * X_MULTIPLIER);
        System.out.println(encoderTicksToInches((int) perpendicularEncoder.getCorrectedVelocity()) * Y_MULTIPLIER);

        parallelEncoder.getCurrentPosition();      // needed in order to get the proper corrected velocity
        perpendicularEncoder.getCurrentPosition(); // see docs

        return Arrays.asList(
                encoderTicksToInches((int) parallelEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches((int) perpendicularEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }


}