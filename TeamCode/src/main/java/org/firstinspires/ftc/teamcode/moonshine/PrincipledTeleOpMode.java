package org.firstinspires.ftc.teamcode.moonshine;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.moonshine.builtin.BlueprintCommand;
import org.firstinspires.ftc.teamcode.moonshine.builtin.ContinuousReuseCommand;
import org.firstinspires.ftc.teamcode.moonshine.builtin.SerialCommand;

public abstract class PrincipledTeleOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        CommandEnv.getInstance().reset();

        ContinuousReuseCommand subsystems = getSubsystems();
        ContinuousReuseCommand controlScheme = getControlScheme();
        SerialCommand initRoutine = getInitRoutine();
        SerialCommand startRoutine = getStartRoutine();

        // INIT
        subsystems.step();
        while(opModeInInit()) {
            subsystems.step();
            initRoutine.step();
            telemetry.update();
        }
        initRoutine.end();

        waitForStart();

        // OPMODE
        while(opModeIsActive()) {
            subsystems.step();
            controlScheme.step();
            startRoutine.step();
            telemetry.update();
        }
        subsystems.end();
        controlScheme.end();
        startRoutine.end();

        CommandEnv.getInstance().reset();
    }

    public abstract ContinuousReuseCommand getSubsystems();
    public abstract SerialCommand getInitRoutine();
    public abstract ContinuousReuseCommand getControlScheme();
    public abstract SerialCommand getStartRoutine();
}
