package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class New5Specimen {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(900);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -62, -Math.PI/2))
                //TODO: FIRST ESPECIMEN
                .strafeTo(new Vector2d(-7,-37))
                .waitSeconds(.4)
                .strafeToLinearHeading(new Vector2d(34, -37), Math.toRadians(45))
                .waitSeconds(.4)
                .turn(-1.8)
                .waitSeconds(.125)
                .turn(1.8)
                .strafeTo(new Vector2d(44, -37))
                .waitSeconds(.4)
                .turn(-1.8)
                .waitSeconds(.4)
                .turn(1)
                .strafeTo(new Vector2d(48, -23))
                .waitSeconds(.4)
                .strafeToLinearHeading(new Vector2d(44, -47), Math.toRadians(-90))
                .waitSeconds(.4)
                .strafeToLinearHeading(new Vector2d(35, -60), Math.toRadians(90))
                .waitSeconds(.4)
                .strafeToLinearHeading(new Vector2d(-2, -37), Math.toRadians(-90))
                .waitSeconds(.4)
                .strafeToLinearHeading(new Vector2d(35, -60), Math.toRadians(90))
                .waitSeconds(.4)
                .strafeToLinearHeading(new Vector2d(0, -37), Math.toRadians(-90))
                .waitSeconds(.4)
                .strafeToLinearHeading(new Vector2d(35, -60), Math.toRadians(90))
                .waitSeconds(.4)
                .strafeToLinearHeading(new Vector2d(2, -37), Math.toRadians(-90))
                .waitSeconds(.4)
                .strafeToLinearHeading(new Vector2d(35, -60), Math.toRadians(90))
                .waitSeconds(.4)
                .strafeToLinearHeading(new Vector2d(4, -37), Math.toRadians(-90))
                .waitSeconds(.4)
                .strafeToLinearHeading(new Vector2d(30, -52), Math.toRadians(-45))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}