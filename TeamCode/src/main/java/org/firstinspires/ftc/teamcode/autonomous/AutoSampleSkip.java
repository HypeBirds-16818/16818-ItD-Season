package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.Climber;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Outtake;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOp;

@Autonomous(name = "TE AMO VALE COV")
public class AutoSampleSkip extends LinearOpMode {

    public double armRaiseTime = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(-24, -58.5, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Outtake outake = new Outtake(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Climber climber = new Climber(hardwareMap);

        climber.init(hardwareMap);
        intake.init(hardwareMap);
        outake.init(hardwareMap);

        TrajectoryActionBuilder firstMovement = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(-50, -51),new TranslationalVelConstraint(70),new ProfileAccelConstraint(-50,50));

        TrajectoryActionBuilder scoreFirstSample = firstMovement.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-59.5, -51), Math.toRadians(45));

        TrajectoryActionBuilder driveToFirstSample = scoreFirstSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-50, -41.7), Math.toRadians(90.00))
                .strafeToConstantHeading(new Vector2d(-55.5, -41.7))
                .strafeToConstantHeading(new Vector2d(-55.5, -34),new TranslationalVelConstraint(60),new ProfileAccelConstraint(-30,30));

        TrajectoryActionBuilder driveToBasketFirst = driveToFirstSample.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-59.5, -51), Math.toRadians(45));

        TrajectoryActionBuilder driveToSecondSample = driveToBasketFirst.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-50, -39.7), Math.toRadians(90.00))
                .strafeToConstantHeading(new Vector2d(-66.5, -39.7))
                .strafeToConstantHeading(new Vector2d(-65.8, -33),new TranslationalVelConstraint(60),new ProfileAccelConstraint(-30,30));

        TrajectoryActionBuilder driveToBasketSecond = driveToSecondSample.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-59.5, -51), Math.toRadians(45));

        TrajectoryActionBuilder park = driveToBasketSecond.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-62.5,-12.13), Math.toRadians(20))
                .strafeToLinearHeading(new Vector2d(-29.76, -12.13), Math.toRadians(0));


        waitForStart();

        Actions.runBlocking(new SequentialAction(climber.setTarget(30), intake.setTargetAction(-60),
                new ParallelAction(
                        climber.updatePIDAction(),
                        intake.updatePIDAction(),

                        new SequentialAction(
                                outake.setMunecaAction(0.17),
                                outake.setGarraAction(TeleOp.GARRA_CERRADA_O),
                                new ParallelAction(
                                        new SequentialAction(
                                                outake.setRotationAction(TeleOp.BRAZO_OUT_SCORING),
                                                // Poner el intake de regreso para tomar

                                                outake.setInnerRotationAction(TeleOp.ROT_OUT_START),
                                                climber.setTarget(3700)
                                        ),
                                        firstMovement.build()
                                ),

                                new SleepAction(0.5),
                                scoreFirstSample.build(),
                                new SleepAction(0.7),
                                outake.setGarraAction(TeleOp.GARRA_ABIERTA_O),
                                intake.setGarraAction(TeleOp.GARRA_ABIERTA_I),
                                new SleepAction(0.5),

                                new ParallelAction(
                                        outake.setRotationAction(TeleOp.BRAZO_OUT_PREINTAKE),
                                        // Poner el intake de regreso para tomar
                                        outake.setInnerRotationAction(TeleOp.ROT_OUT_PRETAKE),
                                        climber.setTarget(10)
                                ),

                                new SleepAction(1),
//
                                new ParallelAction(
                                        driveToFirstSample.build()
                                ),
                                new SequentialAction(
                                        new SleepAction(0.3),
                                        intake.setRotationAction(TeleOp.BRAZO_IN_INTAKING),
                                        intake.setInnerRotationAction(TeleOp.ROT_IN_PREINTAKE),
                                        new SleepAction(1),
                                        intake.setGarraAction(TeleOp.GARRA_CERRADA_I),
                                        new SleepAction(0.7), // remove
                                        intake.setRotationAction(TeleOp.BRAZO_IN_TRANSFER),
                                        intake.setInnerRotationAction(TeleOp.ROT_IN_TRANSFER)
                                ),

                                outake.setRotationAction(TeleOp.BRAZO_OUT_PREINTAKE-0.1),
                                new SleepAction(0.3),
                                intake.setRotationAction(TeleOp.BRAZO_IN_TRANSFER),
                                intake.setInnerRotationAction(TeleOp.ROT_IN_TRANSFER),
                                new SleepAction(0.5),
                                outake.setRotationAction(TeleOp.BRAZO_OUT_TRANSFER+0.1),
                                outake.setInnerRotationAction(TeleOp.ROT_OUT_TRANSFER),
                                outake.setGarraAction(TeleOp.GARRA_ABIERTA_O),
                                new SleepAction(1),
                                outake.setGarraAction(TeleOp.GARRA_CERRADA_O),
                                new SleepAction(0.3),
                                intake.setGarraAction(TeleOp.GARRA_ABIERTA_I),
                                new SleepAction(0.3),
                                new ParallelAction(
                                        new SequentialAction(
                                                outake.setRotationAction(TeleOp.BRAZO_OUT_SCORING),
                                                // Poner el intake de regreso para tomar

                                                outake.setInnerRotationAction(TeleOp.ROT_OUT_START),
                                                climber.setTarget(3700)
                                        ),
                                        new SequentialAction(
                                                new SleepAction(1),
                                                driveToBasketFirst.build()
                                        )

                                ),
                                new SleepAction(1),
                                outake.setGarraAction(TeleOp.GARRA_ABIERTA_O),

                                //
                                new SleepAction(0.5),

                                new ParallelAction(
                                        outake.setRotationAction(TeleOp.BRAZO_OUT_PREINTAKE),
                                        // Poner el intake de regreso para tomar
                                        outake.setInnerRotationAction(TeleOp.ROT_OUT_PRETAKE)
                                ),

                                new SleepAction(1),
                                climber.setTarget(10),
//
                                new ParallelAction(
                                        new SequentialAction(
                                                new SleepAction(0.5)
                                        ),
                                        driveToSecondSample.build()

                                ),
                                new SequentialAction(
                                        new SleepAction(0.3),
                                        intake.setRotationAction(TeleOp.BRAZO_IN_INTAKING),
                                        intake.setInnerRotationAction(TeleOp.ROT_IN_PREINTAKE),
                                        new SleepAction(1),
                                        intake.setGarraAction(TeleOp.GARRA_CERRADA_I),
                                        new SleepAction(0.7), // remove
                                        intake.setRotationAction(TeleOp.BRAZO_IN_TRANSFER),
                                        intake.setInnerRotationAction(TeleOp.ROT_IN_TRANSFER)
                                ),

                                outake.setRotationAction(TeleOp.BRAZO_OUT_PREINTAKE-0.1),
                                new SleepAction(0.3),
                                intake.setRotationAction(TeleOp.BRAZO_IN_TRANSFER),
                                intake.setInnerRotationAction(TeleOp.ROT_IN_TRANSFER),
                                new SleepAction(0.5),
                                outake.setRotationAction(TeleOp.BRAZO_OUT_TRANSFER+0.1),
                                outake.setInnerRotationAction(TeleOp.ROT_OUT_TRANSFER),
                                outake.setGarraAction(TeleOp.GARRA_ABIERTA_O),
                                new SleepAction(1),
                                outake.setGarraAction(TeleOp.GARRA_CERRADA_O),
                                new SleepAction(0.3),
                                intake.setGarraAction(TeleOp.GARRA_ABIERTA_I),
                                new SleepAction(0.3),
                                new ParallelAction(
                                        new SequentialAction(
                                                outake.setRotationAction(TeleOp.BRAZO_OUT_SCORING),
                                                // Poner el intake de regreso para tomar

                                                outake.setInnerRotationAction(TeleOp.ROT_OUT_START),
                                                climber.setTarget(3700)
                                        ),
                                        new SequentialAction(
                                                new SleepAction(1),
                                                driveToBasketSecond.build()
                                        )

                                ),
                                new SleepAction(1),
                                outake.setGarraAction(TeleOp.GARRA_ABIERTA_O),
                                new SleepAction(0.5),

                                new ParallelAction(
                                        outake.setRotationAction(TeleOp.BRAZO_OUT_PREINTAKE),
                                        // Poner el intake de regreso para tomar
                                        outake.setInnerRotationAction(TeleOp.ROT_OUT_PRETAKE),
                                        climber.setTarget(10)
                                ),
                                intake.setRotationAction(TeleOp.BRAZO_IN_IDLE),
                                intake.setGarraAction(TeleOp.GARRA_ABIERTA_I),
                                intake.setInnerRotationAction(TeleOp.ROT_IN_DOWN),
                                new ParallelAction(
                                        new SequentialAction(
                                                outake.setRotationAction(0.68),
                                                // Poner el intake de regreso para tomar
                                                outake.setInnerRotationAction(0.5),
                                                outake.setMunecaAction(0)
                                        ),
                                        park.build()
                                )



                        ))));







    }

}


