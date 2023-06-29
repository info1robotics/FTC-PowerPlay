package org.firstinspires.ftc.teamcode.CommonPackage;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ThreadedIMU {
    private final Object imuLock1 = new Object();
    private final Object imuLock2 = new Object();
    private final IMU imu;
    public static double imuAngle = 0;
    public static double imuVelocity = 0;
    private static ThreadedIMU instance;

    public static ThreadedIMU getInstance() {
        return instance;
    }

    public ThreadedIMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));
        imu.initialize(parameters);
        imu.resetYaw();
        instance = this;
    }

    public void startIMUThread(LinearOpMode opMode) {
        new Thread(() -> {
            System.out.println("start imu thread");
            System.out.println(opMode.isStopRequested());
            System.out.println(opMode.opModeIsActive());
            imu.resetYaw();
            System.out.println("fdsjfsdkj");
            while (!opMode.isStopRequested()) {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                };
                synchronized (imuLock1) {
                    imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                }
                synchronized (imuLock2) {
                    imuVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
                }
            }
        }).start();
    }

    public double getHeading() {
        return imuAngle;
    }
    public double getVelocity() {
        return imuVelocity;
    }
}
