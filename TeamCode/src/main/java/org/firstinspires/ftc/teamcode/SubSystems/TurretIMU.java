package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class TurretIMU {

    private DcMotor motor;
    private BNO055IMU imu;

    public static float maxVelocity = 0.4f;
    public static float maxAcceleration = 0.2f;

    private int targetAngle;
    private int targetReachedTreshold = 30;

    public TurretIMU(HardwareMap hmap) {
        motor = hmap.get(DcMotor.class, "Turret");
        imu = hmap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

        targetAngle = getAngle();
    }

    public void setTargetAngle(int angle) {
        targetAngle = angle;
    }

    public int getAngleBetween(int angle1, int angle2) {
        return Math.abs((angle1 - angle2) % 360 - 180);
    }

    public void update() {
        int currentAngle = getAngle();

        int direction = (int) Math.signum((targetAngle - currentAngle + 360) % 360 - 180);
        System.out.println((targetAngle - currentAngle + 360) % 360);

        if(getAngleBetween(targetAngle, currentAngle) < targetReachedTreshold)
        {
            motor.setPower(0.0);
        } else {

            double velocity = (maxVelocity * Math.min(1.0, (float)getAngleBetween(targetAngle, currentAngle) / 60));
            System.out.println(velocity);

            motor.setPower(velocity * -direction);
        }
    }

    int getTargetAngle() {
        return targetAngle;
    }

    int getAngle() {
        return (int)imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + 180;
    }
}
