package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Climber;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Outtake;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOp;

@Autonomous
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(23.75, -63, Math.toRadians(90.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Outtake outake = new Outtake(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Climber climber = new Climber(hardwareMap);

        climber.init(hardwareMap);
        intake.init(hardwareMap);

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


        waitForStart();


        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        //INIT
                        outake.setBrazoAction(TeleOp.BRAZO_OUT_MEDIO),
                        outake.setRotationAction(TeleOp.ROT_OUT_CERO),
                        outake.setGarraAction(TeleOp.GARRA_ABIERTA_O),
                        intake.setBrazoAction(TeleOp.BRAZO_IN_CERO),
                        intake.setRotationAction(TeleOp.ROT_IN_CERO),
                        intake.setMunecaAction(TeleOp.MUNECA_VER),
                        intake.setGarraAction(TeleOp.GARRA_ABIERTA_I),
                        intake.setTarget(0),
                        climber.setTarget(-50)
                ),
                new SleepAction(1),
                tab1.build()));

    }
}
