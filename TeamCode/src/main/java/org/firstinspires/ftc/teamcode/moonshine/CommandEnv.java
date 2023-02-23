package org.firstinspires.ftc.teamcode.moonshine;

import android.content.Context;
import android.util.Log;

import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.EventLoop;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

public class CommandEnv {
    private static CommandEnv instance;

    private CommandEnv() {

    }
    static public CommandEnv getInstance() {
        if(instance == null)
            instance = new CommandEnv();

        return instance;
    }


    public EventLoop eventLoop;
    public HashMap<String, Object> sharedVars = new HashMap<>();


    @OnCreateEventLoop
    public static void attachEventLoop(Context context, FtcEventLoop eventLoop) {
        getInstance().eventLoop = eventLoop;
        eventLoop.getOpModeManager().registerListener(new OpModeManagerNotifier.Notifications() {
            @Override
            public void onOpModePreInit(OpMode opMode) {

                getInstance().sharedVars.clear();
                Log.d("info1", "Cleared vars!!");
            }

            @Override
            public void onOpModePreStart(OpMode opMode) {

            }

            @Override
            public void onOpModePostStop(OpMode opMode) {
            }
        });

    }

    public void reset() {
        sharedVars.clear();
    }

}
