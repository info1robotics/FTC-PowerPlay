package org.firstinspires.ftc.teamcode.OpModes.Autonomous.CRI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

import org.firstinspires.ftc.teamcode.SubSystems.V3.Controller;

@Autonomous
@Config
public class TestPivot extends LinearOpMode {
    public static int sleep = 530;
    @Override
    public void runOpMode() throws InterruptedException {
        Controller ct = new Controller(this);
        ct.pivot.setCollect();
        waitForStart();
        ct.pivot.setHalf();
        sleep(sleep);
        ct.pivot.setScore();
    }
}
