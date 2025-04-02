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

@Disabled
@Autonomous(name = "UIA")
public class AutoSamples extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(-14.74, -62.90, Math.toRadians(270.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Outtake outake = new Outtake(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Climber climber = new Climber(hardwareMap);

        climber.init(hardwareMap);
        intake.init(hardwareMap);
        outake.init(hardwareMap);

        TrajectoryActionBuilder moveCentimeter = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-14.74, -58.90));

        TrajectoryActionBuilder driveToFirstSample = moveCentimeter.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-11.95, -31.73));



        TrajectoryActionBuilder driveToYellow = driveToFirstSample.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-31.70, -58.90))
                .turnTo(Math.toRadians(180))
                .strafeToConstantHeading(new Vector2d(-49.1, -37.3));


        TrajectoryActionBuilder driveToBasket = driveToYellow.endTrajectory().fresh()
                .turnTo(Math.toRadians(235))
                .strafeToConstantHeading(new Vector2d(-61.5, -61.5));

        TrajectoryActionBuilder park = driveToBasket.endTrajectory().fresh()
                        .strafeToConstantHeading(new Vector2d(-33.67, -10))
                                .turnTo(0)
                .strafeToConstantHeading(new Vector2d(-23.93, -10));




        waitForStart();


        Actions.runBlocking(new SequentialAction(climber.setTarget(30), intake.setTargetAction(-40),
                new ParallelAction(
                        climber.updatePIDAction(),
                        intake.updatePIDAction(),
                        new SequentialAction(
                                moveCentimeter.build(),
                                new SleepAction(0.7),
                                //INIT
                                outake.setRotationAction(TeleOp.BRAZO_OUT_SPECIMEN-0.2),
                                new SleepAction(0.6),
                                outake.setMunecaAction(TeleOp.MUNECA_O_HOR),
                                outake.setGarraAction(TeleOp.GARRA_CERRADA_O),
                                outake.setInnerRotationAction(TeleOp.ROT_OUT_OTTAKE),

//                                intake.setSlidersAction(TeleOp.SLIDER_I_IN),

//                                intake.setMunecaAction(TeleOp.MUNECA_I_HOR),

                                // SPECIMEN OUTAKE
                                new SleepAction(0.5),
                                climber.setTarget(100),
                                outake.setRotationAction(TeleOp.BRAZO_OUT_SCORING),
                                outake.setInnerRotationAction(TeleOp.ROT_OUT_OTLEAVE),
                                new SleepAction(2),
                                outake.setMunecaAction(TeleOp.MUNECA_O_FLIPPED),
                                new SleepAction(0.2),
                                climber.setTarget(5),

                                // MOVE TO FIRST SAMPLE
                                driveToFirstSample.build(),

                                new SleepAction(0.4), // optimize
                                outake.setGarraAction(TeleOp.GARRA_ABIERTA_O),

                                // INIT OUTAKE TO MOVE SAMPLES IN FLOOR
                                new ParallelAction(
                                        driveToYellow.build(),
                                        new SequentialAction(
                                                outake.setRotationAction(TeleOp.BRAZO_OUT_PREINTAKE),
                                                outake.setInnerRotationAction(TeleOp.ROT_OUT_PRETAKE),
                                                outake.setGarraAction(TeleOp.GARRA_ABIERTA_O),
                                                new SleepAction(0.5),
                                                intake.setRotationAction(TeleOp.BRAZO_IN_PREINTAKE),
                                                intake.setGarraAction(TeleOp.GARRA_ABIERTA_I),
                                                intake.setInnerRotationAction(TeleOp.ROT_IN_PREINTAKE)
                                        )
                                ),
                                new SleepAction(0.5),
                                intake.setMunecaAction(TeleOp.MUNECA_I_HOR),
                                intake.setGarraAction(TeleOp.GARRA_ABIERTA_I),

                                outake.setRotationAction(TeleOp.BRAZO_OUT_SCORING)
//                                new SleepAction(0.2),
//                                // CLOSE CLAW
//                                outake.setGarraAction(TeleOp.GARRA_CERRADA_O),
//                                new SleepAction(0.4),
//                                climber.setTarget(TeleOp.CLIMBER_SLIGHTLY+30),
//
//                                // MOVE TO SECOND SAMPLE WHILE MOVING OUTAKE
//                                new ParallelAction(
//                                        new SequentialAction(
//                                                new SleepAction(0.3),
//                                                driveToSecondSample.build()
//                                        ),
//                                        new SequentialAction(
//                                                outake.setRotationAction(TeleOp.BRAZO_OUT_SCORING),
//                                                outake.setInnerRotationAction(TeleOp.ROT_OUT_OTLEAVE),
//                                                new SleepAction(2),
//                                                outake.setMunecaAction(TeleOp.MUNECA_O_FLIPPED),
//                                                new SleepAction(0.2),
//                                                climber.setTarget(5)
//                                        )
//                                ),
//                                new SleepAction(0.4),
//                                outake.setGarraAction(TeleOp.GARRA_ABIERTA_O),
//
//                                // RETURN FOR ANOTHER SAMPLE WHILE RESETING OUTAKE
//                                new ParallelAction(
//                                        firstReturn.build(),
//                                        new SequentialAction(
//                                                outake.setInnerRotationAction(TeleOp.ROT_OUT_OTTAKE),
//                                                new SleepAction(0.5),
//                                                outake.setMunecaAction(TeleOp.MUNECA_O_HOR),
//                                                outake.setRotationAction(TeleOp.BRAZO_OUT_SPECIMEN)
//                                        )
//                                ),
//                                new SleepAction(0.2),
//                                // CLOSE CLAW
//                                outake.setGarraAction(TeleOp.GARRA_CERRADA_O),
//                                new SleepAction(0.4),
//                                climber.setTarget(TeleOp.CLIMBER_SLIGHTLY + 30),
//
//                                // MOVE TO THIRD SAMPLE WHILE MOVING OUTAKE
//                                new ParallelAction(
//                                        driveToThirdSample.build(),
//                                        new SequentialAction(
//                                                outake.setRotationAction(TeleOp.BRAZO_OUT_SCORING),
//                                                outake.setInnerRotationAction(TeleOp.ROT_OUT_OTLEAVE),
//                                                new SleepAction(2),
//                                                outake.setMunecaAction(TeleOp.MUNECA_O_FLIPPED),
//                                                new SleepAction(0.2),
//                                                climber.setTarget(5)
//                                        )
//                                ),
//                                new SleepAction(0.4),
//                                outake.setGarraAction(TeleOp.GARRA_ABIERTA_O),
//
//                                // RETURN FOR ANOTHER SAMPLE WHILE RESETING OUTAKE
//                                new ParallelAction(
//                                        secondReturn.build(),
//                                        new SequentialAction(
//                                                outake.setInnerRotationAction(TeleOp.ROT_OUT_OTTAKE),
//                                                new SleepAction(0.5),
//                                                outake.setMunecaAction(TeleOp.MUNECA_O_HOR),
//                                                outake.setRotationAction(TeleOp.BRAZO_OUT_SPECIMEN)
//                                        )
//                                )
//                                new SleepAction(0.2),
//                                // CLOSE CLAW
//                                outake.setGarraAction(TeleOp.GARRA_CERRADA_O),
//                                new SleepAction(0.4),
//                                climber.setTarget(TeleOp.CLIMBER_SLIGHTLY+30),
//
//                                // MOVE TO FOURTH SAMPLE WHILE MOVING OUTAKE
//                                new ParallelAction(
//                                        driveToFourthSample.build(),
//                                        new SequentialAction(
//                                                outake.setRotationAction(TeleOp.BRAZO_OUT_SCORING),
//                                                outake.setInnerRotationAction(TeleOp.ROT_OUT_OTLEAVE),
//                                                new SleepAction(1),
//                                                outake.setMunecaAction(TeleOp.MUNECA_O_FLIPPED),
//                                                new SleepAction(0.2),
//                                                climber.setTarget(TeleOp.CLIMBER_SLIGHTLY)
//                                        )
//                                ),
//                                new SleepAction(1),
//                                outake.setGarraAction(TeleOp.GARRA_ABIERTA_O),
//
//                                // RETURN FOR ANOTHER SAMPLE WHILE RESETING OUTAKE
//                                new ParallelAction(
//                                        thirdReturn.build(),
//                                        new SequentialAction(
//                                                outake.setInnerRotationAction(TeleOp.ROT_OUT_OTTAKE),
//                                                new SleepAction(0.5),
//                                                outake.setMunecaAction(TeleOp.MUNECA_O_HOR),
//                                                outake.setRotationAction(TeleOp.BRAZO_OUT_SPECIMEN)
//                                        )
//                                ),
//                                new SleepAction(0.2),
//                                // CLOSE CLAW
//                                outake.setGarraAction(TeleOp.GARRA_CERRADA_O),
//                                new SleepAction(0.2),
//                                climber.setTarget(TeleOp.CLIMBER_SLIGHTLY-200),
//
//                                // MOVE TO FIFTH SAMPLE WHILE MOVING OUTAKE
//                                new ParallelAction(
//                                        driveToFifthSample.build(),
//                                        new SequentialAction(
//                                                outake.setRotationAction(TeleOp.BRAZO_OUT_SCORING),
//                                                outake.setInnerRotationAction(TeleOp.ROT_OUT_OTLEAVE),
//                                                new SleepAction(1),
//                                                outake.setMunecaAction(TeleOp.MUNECA_O_FLIPPED),
//                                                new SleepAction(0.2),
//                                                climber.setTarget(TeleOp.CLIMBER_SLIGHTLY)
//                                        )
//                                ),
//                                new SleepAction(1),
//                                outake.setGarraAction(TeleOp.GARRA_ABIERTA_O),
//
//                                // RETURN FOR ANOTHER SAMPLE WHILE RESETING OUTAKE
//                                new ParallelAction(
//                                        fourthReturn.build(),
//                                        new SequentialAction(
//                                                outake.setInnerRotationAction(TeleOp.ROT_OUT_OTTAKE),
//                                                new SleepAction(0.5),
//                                                outake.setMunecaAction(TeleOp.MUNECA_O_HOR),
//                                                outake.setRotationAction(TeleOp.BRAZO_OUT_SPECIMEN)
//                                        )
//                                ),
//                                new SleepAction(0.2),
//                                // CLOSE CLAW
//                                outake.setGarraAction(TeleOp.GARRA_CERRADA_O),
//                                new SleepAction(0.2),
//                                climber.setTarget(TeleOp.CLIMBER_SLIGHTLY-200)
                        ))));


    }
}
