package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(9.32, -62.76, Math.toRadians(90.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(38.78, -12.78), Math.toRadians(90.00))
                .waitSeconds(1)
                .strafeTo(new Vector2d(47.74, -12.64))
                .waitSeconds(1)
                .strafeTo(new Vector2d(47.74, -60.30))
                .waitSeconds(1)
                .strafeTo(new Vector2d(47.74, -9.89))
                .waitSeconds(1)
                .strafeTo(new Vector2d(58.28, -9.89))
                .waitSeconds(1)
                .strafeTo(new Vector2d(58.71, -60.30))
                .waitSeconds(1)
                .strafeTo(new Vector2d(58.28, -12.64))
                .waitSeconds(1)
                .strafeTo(new Vector2d(62.90, -12.64))
                .waitSeconds(1)
                .strafeTo(new Vector2d(62.90, -60.30));


        Action trajectoryToDrive = tab1.build();

        Actions.runBlocking(new SequentialAction(trajectoryToDrive));

    }
}
