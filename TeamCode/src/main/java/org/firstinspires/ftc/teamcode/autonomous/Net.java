package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Climber;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Outtake;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOp;
import org.opencv.core.Mat;

@Autonomous(name = "***** NET *****")
public class Net extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(-24, -62, Math.PI * 2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        //TODO      UNO
        TrajectoryActionBuilder moveToNet = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-54, -52), Math.toRadians(52));

        //TODO      DOS
        TrajectoryActionBuilder driveToFirstSample = moveToNet.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-47, -45), Math.toRadians(90));

        //TODO      TRES
        TrajectoryActionBuilder firstReturn = driveToFirstSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-54, -52), Math.toRadians(52));

        //TODO      CUATRO
        TrajectoryActionBuilder driveToSecondSample = firstReturn.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-58, -41), Math.toRadians(90));

        //TODO      CINCO
        TrajectoryActionBuilder secondReturn = driveToSecondSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-54, -52), Math.toRadians(52));

        //TODO      SEIS
        TrajectoryActionBuilder driveToThirdSample = secondReturn.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-48, -24), Math.toRadians(180));

        //TODO      SIETE
        TrajectoryActionBuilder thirdReturn = driveToThirdSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-54, -52), Math.toRadians(52));

        //TODO      OCHO
        TrajectoryActionBuilder driveToSub = thirdReturn.endTrajectory().fresh()
                .setTangent(Math.PI /3)
                .splineToLinearHeading(new Pose2d(-39, -10, Math.PI * 2), Math.PI/2)
                .strafeTo(new Vector2d(-25, -10));

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                moveToNet.build(),
                new SleepAction(0.4),
                driveToFirstSample.build(),
                new SleepAction(0.6),
                firstReturn.build(),
                new SleepAction(0.4),
                driveToSecondSample.build(),
                new SleepAction(0.6),
                secondReturn.build(),
                new SleepAction(0.4),
                driveToThirdSample.build(),
                new SleepAction(0.6),
                thirdReturn.build(),
                new SleepAction(0.4),
                driveToSub.build()
        ));
    }
}
