package org.firstinspires.ftc.teamcode.RoadRunner.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.CommonPackage.ThreadedIMU;
import org.firstinspires.ftc.teamcode.RoadRunner.util.Encoder;

import java.util.Arrays;
import java.util.List;

//@Config
public class StandardTrackingWheelLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.98; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = -3.9566; // X is the up and down direction
    public static double PARALLEL_Y = 1.9094; // Y is the strafe direction
    // -100.5 mm x up down
    // 48.5 mm y strafe

    public static double PERPENDICULAR_X = -2.6968;
    public static double PERPENDICULAR_Y = -1.1259;
    // -68.15 mm x up down
    // - 28.6 mm y strafe

    public static double X_MULTIPLIER = 0.9847477886 * 1.01116778644 * 0.9950930854 * 1.001094530019 * 1.001112347052 * 0.9965563442; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 0.9883093979 * 0.996307022 * 0.9976984206 * 0.9968466418 * 1.006781230913 * 0.9932782872 * 1.00355369514; // Multiplier in the Y direction

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    public final Encoder parallelEncoder;
    public final Encoder perpendicularEncoder;
    private final ThreadedIMU imu;

    public static StandardTrackingWheelLocalizer instance = null;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        imu = ThreadedIMU.getInstance();

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BR"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FR"));
        perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);
        instance = this;
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return imu.getHeading();
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
        return imu.getVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
//        System.out.println("WHEEL VELOCITIES");
//        System.out.println(encoderTicksToInches((int) parallelEncoder.getCorrectedVelocity()) * X_MULTIPLIER);
//        System.out.println(encoderTicksToInches((int) perpendicularEncoder.getCorrectedVelocity()) * Y_MULTIPLIER);

        parallelEncoder.getCurrentPosition();      // needed in order to get the proper corrected velocity
        perpendicularEncoder.getCurrentPosition(); // see docs

        return Arrays.asList(
                encoderTicksToInches((int) parallelEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches((int) perpendicularEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }


}