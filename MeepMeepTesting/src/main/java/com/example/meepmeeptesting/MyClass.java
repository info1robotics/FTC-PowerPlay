package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(1600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(50, 30, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-88.27, 59.5))
                                .lineToLinearHeading(new Pose2d(
                                        -26,
                                        55,
                                        Math.toRadians(-90)
                                ))
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(
                                        -12,
                                        59.5
                                ))
                                .setReversed(true)
                                .lineToConstantHeading(new Vector2d(
                                        -12,
                                        59.5 + 24 + 24 + 12
                                ))
                                .setReversed(true)
                                .lineToConstantHeading(new Vector2d(
                                        0,
                                        59.5 + 24 + 24 + 12
                                ))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(1.0f)
                .addEntity(myBot)
                .start();
    }
}