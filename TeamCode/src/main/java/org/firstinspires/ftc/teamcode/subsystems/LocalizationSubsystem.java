package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.teamcode.moonshine.Subsystem;

public class LocalizationSubsystem extends Subsystem {

    private final Pose2d startPosition;
    Localizer localizer;

    public LocalizationSubsystem(Localizer localizer, Pose2d startPosition) {
        this.localizer = localizer;
        this.startPosition = startPosition;
    }

    @Override
    protected void onStart() {
        localizer.setPoseEstimate(startPosition);
    }

    @Override
    protected void onTick() {
        localizer.update();
    }

    @Override
    protected void onEnd() {

    }

    public Pose2d getCurrentPose() { return localizer.getPoseEstimate(); }

}
