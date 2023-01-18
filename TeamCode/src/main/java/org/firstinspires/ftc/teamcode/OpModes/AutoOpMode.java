package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Tasks.Task;

public abstract class AutoOpMode extends LinearOpMode {
    public Task task;
    public SampleMecanumDrive drive;
    @Override
    public final void runOpMode() throws InterruptedException {
        onInit();
        drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        onStart();
        task.start(this);
        while(opModeIsActive() && task.isRunning()) {
            onLoop();
            task.tick();
        }
    }

    public void onInit() {};
    public void onStart() {};
    public void onLoop() {};
}
