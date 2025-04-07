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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.Climber;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Outtake;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOp;

@Autonomous(name = "SIGMA BOI")
public class AutoSample extends LinearOpMode {

    public double armRaiseTime = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(-15, -62.80, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Outtake outake = new Outtake(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Climber climber = new Climber(hardwareMap);

        climber.init(hardwareMap);
        intake.init(hardwareMap);
        outake.init(hardwareMap);

        TrajectoryActionBuilder scoreFirstSample = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-5, -31.73));

        TrajectoryActionBuilder driveToFirstSample = scoreFirstSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-16.49, -57.61), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-49, -45.77), Math.toRadians(90.00));

        TrajectoryActionBuilder driveToBasketFirst = driveToFirstSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-58.5, -58.5), Math.toRadians(225));

        TrajectoryActionBuilder driveToSecondSample = driveToBasketFirst.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-59.5, -45.77), Math.toRadians(90));

        TrajectoryActionBuilder driveToBasketSecond = driveToSecondSample.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-58.5, -58.5), Math.toRadians(225));

        TrajectoryActionBuilder driveToPark = driveToBasketSecond.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(-49.33, -26.52, Math.toRadians(180.00)), Math.toRadians(180.00));

        TrajectoryActionBuilder driveToBasketThird = driveToPark.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-58.5, -58.5), Math.toRadians(225));

        TrajectoryActionBuilder park = driveToBasketThird.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(-23.76, -12.13, Math.toRadians(0.00)), Math.toRadians(0.00));


        waitForStart();

        Actions.runBlocking(new SequentialAction(climber.setTarget(30), intake.setTargetAction(-40),
                new ParallelAction(
                        climber.updatePIDAction(),
                        intake.updatePIDAction(),

                new SequentialAction(
//                                moveCentimeter.build(),

                        // SPECIMEN OUTAKE
                        outake.setRotationAction(TeleOp.BRAZO_OUT_SCORING-0.03),
                        outake.setInnerRotationAction(TeleOp.ROT_OUT_OTLEAVE),
                        new SleepAction(armRaiseTime),
                        outake.setMunecaAction(TeleOp.MUNECA_O_FLIPPED),
                        new SleepAction(0.2),
                        climber.setTarget(5),

                        // MOVE TO FIRST SAMPLE
                        driveToFirstSample.build(),

                        new SleepAction(0.3), // optimize
                        outake.setGarraAction(TeleOp.GARRA_ABIERTA_O),

                        // MOVE TO BASKET
                        new ParallelAction(
                                driveToFirstSample.build(),
                                new SequentialAction(
                                        new SleepAction(1),
                                        outake.setInnerRotationAction(TeleOp.ROT_OUT_START),
                                        outake.setMunecaAction(TeleOp.MUNECA_O_HOR),
                                        outake.setRotationAction(TeleOp.BRAZO_OUT_IDLE),
                                        new SleepAction(1),
                                        intake.setTargetAction(250)
                                )
                        ),
                        new SleepAction(3),
                        outake.setInnerRotationAction(TeleOp.ROT_OUT_PRETAKE),
                        outake.setRotationAction(TeleOp.BRAZO_OUT_PREINTAKE),
                        outake.setGarraAction(TeleOp.GARRA_ABIERTA_O),
                        intake.setGarraAction(TeleOp.GARRA_ABIERTA_I),
                        new SleepAction(0.5),
                        intake.setRotationAction(TeleOp.BRAZO_IN_INTAKING),
                        intake.setInnerRotationAction(TeleOp.ROT_IN_PREINTAKE),
                        new SleepAction(0.5),
                        intake.setGarraAction(TeleOp.GARRA_CERRADA_I),
                        intake.setRotationAction(TeleOp.BRAZO_IN_TRANSFER),
                        intake.setInnerRotationAction(TeleOp.ROT_IN_TRANSFER),
                        outake.setRotationAction(TeleOp.BRAZO_OUT_PREINTAKE-0.07),
                        new SleepAction(0.2),
                        intake.setTargetAction(-100),
                        new SleepAction(0.5),
                        intake.setModeAction(DcMotor.RunMode.STOP_AND_RESET_ENCODER),
                        intake.setModeAction(DcMotor.RunMode.RUN_USING_ENCODER),
                        intake.setTargetAction(0),
                        new SleepAction(0.1),
                        outake.setRotationAction(TeleOp.BRAZO_OUT_TRANSFER),
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
                                        climber.setTarget(2600)
                                ),
                                driveToBasketFirst.build()
                        ),
                        new SleepAction(0.4),
                        outake.setGarraAction(TeleOp.GARRA_ABIERTA_O)


        ))));







        }

    }


