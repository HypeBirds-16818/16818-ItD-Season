package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Climber;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.PIDActionIntake;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOp;
import org.opencv.core.Mat;

@Autonomous
public class AutoSample extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(-33.34, -62.80, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Outtake outake = new Outtake(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Climber climber = new Climber(hardwareMap);

        climber.init(hardwareMap);
        intake.init(hardwareMap);

        TrajectoryActionBuilder driveToFirstSample = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-49, -45.77), Math.toRadians(90.00));

        TrajectoryActionBuilder driveToBasketFirst = driveToFirstSample.endTrajectory().fresh()
                .turnTo(Math.toRadians(225))
                .strafeToLinearHeading(new Vector2d(-58.5, -58.5), Math.toRadians(225));

        TrajectoryActionBuilder driveToSecondSample = driveToBasketFirst.endTrajectory().fresh()
                .turnTo(Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-59.5, -45.77), Math.toRadians(90.00));

        TrajectoryActionBuilder driveToBasketSecond = driveToSecondSample.endTrajectory().fresh()
                .turnTo(Math.toRadians(225))
                .strafeToLinearHeading(new Vector2d(-58.5, -58.5), Math.toRadians(225));

        TrajectoryActionBuilder driveToPark = driveToBasketSecond.endTrajectory().fresh()
                .turnTo(Math.toRadians(90))
                .splineTo(new Vector2d(-24.48, -10.04), Math.toRadians(-0.83));



        waitForStart();

        Actions.runBlocking(new SequentialAction(intake.setTarget(0), climber.setTarget(-50),
                new ParallelAction(
                    intake.updatePIDAction(), climber.updatePIDAction(),
                    new SequentialAction(
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
                        new ParallelAction(
                                //IDLE
                            outake.setBrazoAction(TeleOp.BRAZO_OUT_MEDIO),
                            outake.setGarraAction(TeleOp.GARRA_CERRADA_O),
                            intake.setBrazoAction(TeleOp.BRAZO_IN_CERO),
                            intake.setRotationAction(TeleOp.ROT_IN_CERO)
                        ),
                        driveToFirstSample.build(),
                        new SleepAction(1),
                        new ParallelAction(
                                //START INTAKING
                            intake.setGarraAction(TeleOp.GARRA_ABIERTA_I),
                            outake.setGarraAction(TeleOp.GARRA_ABIERTA_O)
                        ),

                        new SleepAction(0.2),
                        intake.setTarget(TeleOp.SLIDER_I_OUT),
                        new SleepAction(0.5),
                        new SequentialAction(
                            intake.setBrazoAction(TeleOp.BRAZO_IN_NOVENTA),
                            intake.setMunecaAction(TeleOp.MUNECA_VER)
                        ),
                        new SleepAction(0.5),
                        new SequentialAction(
                                // BAJA INTAKE
                            intake.setRotationAction(TeleOp.ROT_IN_NOVENTA),
                            intake.setBrazoAction(TeleOp.BRAZO_IN_CIENTOCHENTA)
                        ),
                        new SleepAction(3),
                            // CLOSE CLAW
                        intake.setGarraAction(TeleOp.GARRA_CERRADA_I),
                        new SequentialAction(
                                // PRETRANSFER
                            intake.setBrazoAction(TeleOp.BRAZO_IN_NOVENTA),
                            intake.setMunecaAction(TeleOp.MUNECA_HOR),
                            new SleepAction(0.2),
                            intake.setTarget(TeleOp.SLIDER_I_IN)
                        ),
                        new SleepAction(1),
                        new SequentialAction(
                                //TRANSFER
                            intake.setRotationAction(TeleOp.ROT_IN_CERO),
                            outake.setBrazoAction(TeleOp.BRAZO_OUT_MEDIO),
                            new SleepAction(0.7),
                            intake.setBrazoAction(TeleOp.BRAZO_IN_CERO),
                            new SleepAction(0.7),
                            outake.setBrazoAction(TeleOp.BRAZO_OUT_ABAJO),
                            new SleepAction(0.7),
                            outake.setGarraAction(TeleOp.GARRA_CERRADA_O),
                            new SleepAction(0.7),
                            intake.setGarraAction(TeleOp.GARRA_ABIERTA_I)
                        ),
                        new ParallelAction(
                            driveToBasketFirst.build(),
                            new SequentialAction(
                                climber.setTarget(TeleOp.CLIMBER_ABAJO),
                                outake.setBrazoAction(TeleOp.BRAZO_OUT_MEDIO),
                                new SleepAction(0.7),
                                outake.setRotationAction(TeleOp.ROT_OUT_CIENTOCHENTA),
                                new SleepAction(1),
                                outake.setBrazoAction(TeleOp.BRAZO_OUT_ARRIBA-0.05),
                                new SleepAction(0.5),
                                climber.setTarget(TeleOp.CLIMBER_S_BASKET)
                            )
                        ),
                        new SleepAction(1),
                        outake.setGarraAction(TeleOp.GARRA_ABIERTA_O),
                        new SleepAction(1),
                        new ParallelAction(
                            driveToSecondSample.build(),
                            new SequentialAction(
                                climber.setTarget(TeleOp.CLIMBER_ABAJO),
                                outake.setBrazoAction(TeleOp.BRAZO_OUT_ARRIBA),
                                new SleepAction(0.5),
                                climber.setTarget(TeleOp.CLIMBER_SLIGHTLY - 600),
                                new SleepAction(0.3),
                                outake.setBrazoAction(TeleOp.BRAZO_OUT_MEDIO),
                                new SleepAction(0.3),
                                outake.setRotationAction(TeleOp.ROT_OUT_CERO),
                                new SleepAction(0.7),
                                outake.setBrazoAction(TeleOp.BRAZO_OUT_MEDIO),
                                outake.setGarraAction(TeleOp.GARRA_CERRADA_O),
                                intake.setBrazoAction(TeleOp.BRAZO_IN_CERO),
                                intake.setRotationAction(TeleOp.ROT_IN_CERO),
                                climber.setTarget(TeleOp.CLIMBER_SLIGHTLY)
                            )
                        ),
                        new SleepAction(0.3),
                        new ParallelAction(
                                //START INTAKING
                                intake.setGarraAction(TeleOp.GARRA_ABIERTA_I),
                                outake.setGarraAction(TeleOp.GARRA_ABIERTA_O)
                        ),

                        new SleepAction(0.2),
                        intake.setTarget(TeleOp.SLIDER_I_OUT),
                        new SleepAction(0.5),
                        new SequentialAction(
                                intake.setBrazoAction(TeleOp.BRAZO_IN_NOVENTA),
                                intake.setMunecaAction(TeleOp.MUNECA_VER)
                        ),
                        new SleepAction(0.5),
                        new SequentialAction(
                                // BAJA INTAKE
                                intake.setRotationAction(TeleOp.ROT_IN_NOVENTA),
                                intake.setBrazoAction(TeleOp.BRAZO_IN_CIENTOCHENTA)
                        ),
                        new SleepAction(3),
                        // CLOSE CLAW
                        intake.setGarraAction(TeleOp.GARRA_CERRADA_I),
                        new SequentialAction(
                                // PRETRANSFER
                                intake.setBrazoAction(TeleOp.BRAZO_IN_NOVENTA),
                                intake.setMunecaAction(TeleOp.MUNECA_HOR),
                                new SleepAction(0.2),
                                intake.setTarget(TeleOp.SLIDER_I_IN)
                        ),
                        new SleepAction(1),
                        new SequentialAction(
                                //TRANSFER
                                intake.setRotationAction(TeleOp.ROT_IN_CERO),
                                outake.setBrazoAction(TeleOp.BRAZO_OUT_MEDIO),
                                new SleepAction(0.7),
                                intake.setBrazoAction(TeleOp.BRAZO_IN_CERO),
                                new SleepAction(0.7),
                                outake.setBrazoAction(TeleOp.BRAZO_OUT_ABAJO),
                                new SleepAction(0.7),
                                outake.setGarraAction(TeleOp.GARRA_CERRADA_O),
                                new SleepAction(0.7),
                                intake.setGarraAction(TeleOp.GARRA_ABIERTA_I)
                        ),
                        new ParallelAction(
                                driveToBasketSecond.build(),
                                new SequentialAction(
                                        climber.setTarget(TeleOp.CLIMBER_ABAJO),
                                        outake.setBrazoAction(TeleOp.BRAZO_OUT_MEDIO),
                                        new SleepAction(0.7),
                                        outake.setRotationAction(TeleOp.ROT_OUT_CIENTOCHENTA),
                                        new SleepAction(1),
                                        outake.setBrazoAction(TeleOp.BRAZO_OUT_ARRIBA-0.05),
                                        new SleepAction(0.5),
                                        climber.setTarget(TeleOp.CLIMBER_S_BASKET)
                                )
                        ),
                        new SleepAction(1),
                        outake.setGarraAction(TeleOp.GARRA_ABIERTA_O),
                        new ParallelAction(
                                driveToPark.build(),
                                new SequentialAction(
                                        climber.setTarget(TeleOp.CLIMBER_ABAJO),
                                        outake.setBrazoAction(TeleOp.BRAZO_OUT_ARRIBA),
                                        new SleepAction(0.5),
                                        climber.setTarget(TeleOp.CLIMBER_SLIGHTLY - 600),
                                        new SleepAction(0.3),
                                        outake.setBrazoAction(TeleOp.BRAZO_OUT_MEDIO),
                                        new SleepAction(0.3),
                                        outake.setRotationAction(TeleOp.ROT_OUT_CERO),
                                        new SleepAction(0.7),
                                        outake.setBrazoAction(TeleOp.BRAZO_OUT_MEDIO),
                                        outake.setGarraAction(TeleOp.GARRA_CERRADA_O),
                                        intake.setBrazoAction(TeleOp.BRAZO_IN_CERO),
                                        intake.setRotationAction(TeleOp.ROT_IN_CERO),
                                        climber.setTarget(TeleOp.CLIMBER_SLIGHTLY)
                                )

                    )
        )

        )));







        }

    }


