package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.lang.invoke.VolatileCallSite;

public class MyClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(50, 30, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                drive.trajectorySequenceBuilder(new Pose2d(-37.5, -62, Math.toRadians(90)))
                        .lineTo(new Vector2d(-38.5, -20))
                        .lineTo(new Vector2d(-38.5, -8))
                        .turn(Math.toRadians(90))
                        .lineTo(new Vector2d(-58.25, -9.5))
                        .lineTo(new Vector2d(0, -10))
                        .lineTo(new Vector2d(-58.25, -9.5))
                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
            .setDarkMode(true)
            .setBackgroundAlpha(1.0f)
            .addEntity(myBot)
            .start();
    }
}