package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class fiveSpecimen {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(900);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-24, -62, Math.PI*2))

                //TODO      UNO
                .strafeToLinearHeading(new Vector2d(-54, -52), Math.toRadians(52))
                /*.strafeToConstantHeading(
                        new Vector2d(-54, -55),
                        new TranslationalVelConstraint(100),
                        new ProfileAccelConstraint(-100, 100)
                )*/
                .waitSeconds(.4)
                //TODO      DOS
                .strafeToLinearHeading(new Vector2d(-47, -45), Math.toRadians(90))
                .waitSeconds(.6)
                //TODO      TRES
                .strafeToLinearHeading(new Vector2d(-54, -52), Math.toRadians(52))
                .waitSeconds(.4)
                //TODO      CUATRO
                .strafeToLinearHeading(new Vector2d(-58, -41), Math.toRadians(90))
                .waitSeconds(.6)
                //TODO      CINCO
                .strafeToLinearHeading(new Vector2d(-54, -52), Math.toRadians(52))
                .waitSeconds(.4)
                //TODO      SEIS
                .strafeToLinearHeading(new Vector2d(-48, -24), Math.toRadians(180))
                .waitSeconds(.6)
                //TODO      SIETE
                .strafeToLinearHeading(new Vector2d(-54, -52), Math.toRadians(52))
                .waitSeconds(.4)
                //TODO      OCHO
                .setTangent(Math.PI /3)
                .splineToLinearHeading(new Pose2d(-39, -10, Math.PI * 2), Math.PI/2)
                .strafeTo(new Vector2d(-25, -10))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}