package org.firstinspires.ftc.teamcode.moonshine.builtin;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.moonshine.Command;
import org.firstinspires.ftc.teamcode.moonshine.annotations.RequireHardware;

public class SleepCommand extends Command {
    long startTime;
    final long durationMs;

    DcMotor ad;

    public SleepCommand(long durationMs) {
        this.durationMs = durationMs;
    }

    @Override
    protected void onStart() {
        startTime = System.currentTimeMillis();
    }
    @Override
    protected void onTick() {
        if(startTime + durationMs > System.currentTimeMillis())
            end();
    }
    @Override
    protected void onEnd() {

    }
}
