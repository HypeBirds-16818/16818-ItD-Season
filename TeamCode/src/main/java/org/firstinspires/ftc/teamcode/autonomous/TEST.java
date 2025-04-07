package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Autonomous(name = "TEST")
public final class TEST extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(0, -62, -Math.PI / 2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Action net = drive.actionBuilder(startPose)
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
                .build();

        waitForStart();

        Actions.runBlocking(new ParallelAction(
                net
        ));

        telemetry.addLine("Autonomous Complete!");
        telemetry.update();
    }
}