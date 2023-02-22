package org.firstinspires.ftc.teamcode.moonshine;

import android.content.Context;

import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.eventloop.EventLoop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;

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
                getInstance().sharedVars = new HashMap<>();
            }

            @Override
            public void onOpModePreStart(OpMode opMode) {

            }

            @Override
            public void onOpModePostStop(OpMode opMode) {

            }
        });
    }



}
