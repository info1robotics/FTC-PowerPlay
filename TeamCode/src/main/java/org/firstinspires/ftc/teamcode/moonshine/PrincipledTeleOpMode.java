package org.firstinspires.ftc.teamcode.moonshine;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.moonshine.builtin.BlueprintCommand;
import org.firstinspires.ftc.teamcode.moonshine.builtin.ContinuousReuseCommand;
import org.firstinspires.ftc.teamcode.moonshine.builtin.SerialCommand;

public abstract class PrincipledTeleOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ContinuousReuseCommand subsystems = getSubsystems();
        ContinuousReuseCommand controlScheme = getControlScheme();
        SerialCommand initRoutine = getInitRoutine();

        // INIT
        while(opModeInInit()) {
            subsystems.step();
            initRoutine.step();
        }
        subsystems.end();
        initRoutine.end();


        // OPMODE
        subsystems.reuse();
        while(opModeIsActive()) {
            subsystems.step();
            controlScheme.step();
        }
        subsystems.end();
        controlScheme.end();
    }

    public abstract ContinuousReuseCommand getSubsystems();
    public abstract SerialCommand getInitRoutine();
    public abstract ContinuousReuseCommand getControlScheme();
}
