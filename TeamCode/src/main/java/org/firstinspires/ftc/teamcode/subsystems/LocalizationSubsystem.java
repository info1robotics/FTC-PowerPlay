package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import org.firstinspires.ftc.teamcode.moonshine.CommandEnv;
import org.firstinspires.ftc.teamcode.moonshine.Subsystem;

public class LocalizationSubsystem extends Subsystem {

    private final Pose2d startPosition;
    StandardTrackingWheelLocalizer localizer;

    public LocalizationSubsystem(Pose2d startPosition) {
        this.startPosition = startPosition;
    }

    @Override
    protected void onStart() {
        localizer = new StandardTrackingWheelLocalizer(
            CommandEnv.getInstance().eventLoop.getOpModeManager().getHardwareMap()
        );
        localizer.setPoseEstimate(startPosition);
    }

    @Override
    protected void onTick() {
        localizer.update();
    }

    @Override
    protected void onEnd() {

    }

}
