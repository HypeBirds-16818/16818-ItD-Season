package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Climber;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Outtake;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOp;

@Autonomous(name = "SkibidiSigmaPomni")
public class AutoTest extends LinearOpMode {

    public double armRaiseTime = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(8.74, -62.90, Math.toRadians(270.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Outtake outake = new Outtake(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Climber climber = new Climber(hardwareMap);

        climber.init(hardwareMap);
        intake.init(hardwareMap);
        outake.init(hardwareMap);

        TrajectoryActionBuilder moveCentimeter = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(8.74, -58.90));

        TrajectoryActionBuilder driveToFirstSample = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(1, -27.73));

        TrajectoryActionBuilder moveAllSamples = driveToFirstSample.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(1,-40.73))
                .strafeToConstantHeading(new Vector2d(36.76, -42.59))
                .strafeToConstantHeading(new Vector2d(36.76, -14),new TranslationalVelConstraint(100),new ProfileAccelConstraint(-60,90)) // start moving
                .strafeToConstantHeading(new Vector2d(46.60, -14),new TranslationalVelConstraint(100),new ProfileAccelConstraint(-60,90))
                .strafeToConstantHeading(new Vector2d(46.60, -58.5),new TranslationalVelConstraint(100),new ProfileAccelConstraint(-60,90))
                .strafeToConstantHeading(new Vector2d(46.60, -14),new TranslationalVelConstraint(100),new ProfileAccelConstraint(-60,90))
                .strafeToConstantHeading(new Vector2d(53.99, -14),new TranslationalVelConstraint(100),new ProfileAccelConstraint(-60,90))
                .strafeToConstantHeading(new Vector2d(5.99, -58.5),new TranslationalVelConstraint(100),new ProfileAccelConstraint(-60,90))
                .strafeToConstantHeading(new Vector2d(36.6, -58.5))
//                .strafeToConstantHeading(new Vector2d(51.99, -13))
//                .strafeToConstantHeading(new Vector2d(60.9, -13))
//                .strafeToConstantHeading(new Vector2d(60.9, -60))
//                .strafeToConstantHeading(new Vector2d(55.9, -50))
                .strafeToConstantHeading(new Vector2d(36.6, -64.90));

        TrajectoryActionBuilder driveToSecondSample = moveAllSamples.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-5, -27.7));

        TrajectoryActionBuilder firstReturn = driveToSecondSample.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(5,-40))
                .strafeToConstantHeading(new Vector2d(36.6, -58.5))
//
                .strafeToConstantHeading(new Vector2d(36.6, -63.0));

        TrajectoryActionBuilder driveToThirdSample = firstReturn.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-3, -26.7));

        TrajectoryActionBuilder secondReturn = driveToThirdSample.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(3,-40))
                .strafeToConstantHeading(new Vector2d(36.6, -58.5))
//
                .strafeToConstantHeading(new Vector2d(36.6, -63.70));

        TrajectoryActionBuilder driveToFourthSample = secondReturn.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-1, -25.7));

        TrajectoryActionBuilder thirdReturn = driveToFourthSample.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(3,-40))
                .strafeToConstantHeading(new Vector2d(36.6, -57.5))
//
                .strafeToConstantHeading(new Vector2d(36.6, -63.70));

        TrajectoryActionBuilder driveToFifthSample = thirdReturn.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(3, -27.7));

        TrajectoryActionBuilder fourthReturn = driveToFifthSample.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(36.60, -63.5));



        waitForStart();


        Actions.runBlocking(new SequentialAction(climber.setTarget(30), intake.setTargetAction(-20),
                new ParallelAction(
                        climber.updatePIDAction(),
                                intake.updatePIDAction(),
                        new SequentialAction(

                                // SPECIMEN OUTAKE
                                outake.setRotationAction(TeleOp.BRAZO_OUT_SCORING),
                                outake.setInnerRotationAction(TeleOp.ROT_OUT_OTLEAVE),
                                new SleepAction(armRaiseTime),

                                new SleepAction(0.2),
                                climber.setTarget(5),
                                outake.setMunecaAction(TeleOp.MUNECA_O_FLIPPED),

                                // MOVE TO FIRST SAMPLE
                                driveToFirstSample.build(),

                                new SleepAction(0.3), // optimize
                                outake.setGarraAction(TeleOp.GARRA_ABIERTA_O),

                                // INIT OUTAKE TO MOVE SAMPLES IN FLOOR
                                new ParallelAction(
                                        moveAllSamples.build(),
                                        new SequentialAction(
                                                new SleepAction(1),
                                                outake.setInnerRotationAction(TeleOp.ROT_OUT_OTTAKE),
                                                outake.setMunecaAction(TeleOp.MUNECA_O_HOR),
                                                outake.setRotationAction(TeleOp.BRAZO_OUT_SPECIMEN)
                                        )
                                ),
                                new SleepAction(0.2),
                                // CLOSE CLAW
                                outake.setGarraAction(TeleOp.GARRA_CERRADA_O),
                                new SleepAction(0.4),
                                climber.setTarget(TeleOp.CLIMBER_SLIGHTLY+30),

                                // MOVE TO SECOND SAMPLE WHILE MOVING OUTAKE
                                new ParallelAction(
                                        new SequentialAction(
                                                new SleepAction(0.3),
                                                driveToSecondSample.build()
                                        ),
                                        new SequentialAction(
                                                outake.setRotationAction(TeleOp.BRAZO_OUT_SCORING),
                                                outake.setInnerRotationAction(TeleOp.ROT_OUT_OTLEAVE),
                                                new SleepAction(armRaiseTime),
                                                outake.setMunecaAction(TeleOp.MUNECA_O_FLIPPED),
                                                climber.setTarget(5)
                                        )
                                ),
                                new SleepAction(0.3),
                                outake.setGarraAction(TeleOp.GARRA_ABIERTA_O),
                                new SleepAction(0.3),
                                outake.setRotationAction(TeleOp.BRAZO_OUT_SCORING-0.08),

                                // RETURN FOR ANOTHER SAMPLE WHILE RESETING OUTAKE
                                new ParallelAction(
                                        firstReturn.build(),
                                        new SequentialAction(
                                                outake.setInnerRotationAction(TeleOp.ROT_OUT_OTTAKE),
                                                new SleepAction(0.4),
                                                outake.setMunecaAction(TeleOp.MUNECA_O_HOR),
                                                outake.setRotationAction(TeleOp.BRAZO_OUT_SPECIMEN)
                                        )
                                ),
                                new SleepAction(0.2),
                                // CLOSE CLAW
                                outake.setGarraAction(TeleOp.GARRA_CERRADA_O),
                                new SleepAction(0.4),
                                climber.setTarget(TeleOp.CLIMBER_SLIGHTLY + 30),

                                // MOVE TO THIRD SAMPLE WHILE MOVING OUTAKE
                                new ParallelAction(
                                        new SequentialAction(
                                                new SleepAction(0.3),
                                                driveToThirdSample.build()
                                        ),
                                        new SequentialAction(
                                                outake.setRotationAction(TeleOp.BRAZO_OUT_SCORING),
                                                outake.setInnerRotationAction(TeleOp.ROT_OUT_OTLEAVE),
                                                new SleepAction(armRaiseTime),
                                                outake.setMunecaAction(TeleOp.MUNECA_O_FLIPPED),
                                                climber.setTarget(5)
                                        )
                                ),
                                new SleepAction(0.4),
                                outake.setGarraAction(TeleOp.GARRA_ABIERTA_O),
                                new SleepAction(0.3),
                                outake.setRotationAction(TeleOp.BRAZO_OUT_SCORING-0.08),

                                // RETURN FOR ANOTHER SAMPLE WHILE RESETING OUTAKE
                                new ParallelAction(
                                        secondReturn.build(),
                                        new SequentialAction(
                                                outake.setInnerRotationAction(TeleOp.ROT_OUT_OTTAKE),
                                                new SleepAction(0.4),
                                                outake.setMunecaAction(TeleOp.MUNECA_O_HOR),
                                                outake.setRotationAction(TeleOp.BRAZO_OUT_SPECIMEN)
                                        )
                                ),
                                new SleepAction(0.2),
                                // CLOSE CLAW
                                outake.setGarraAction(TeleOp.GARRA_CERRADA_O),
                                new SleepAction(0.4),
                                climber.setTarget(TeleOp.CLIMBER_SLIGHTLY+30),

                                // MOVE TO FOURTH SAMPLE WHILE MOVING OUTAKE
                                new ParallelAction(
                                        new SequentialAction(
                                                new SleepAction(0.3),
                                                driveToFourthSample.build()
                                        ),
                                        new SequentialAction(
                                                outake.setRotationAction(TeleOp.BRAZO_OUT_SCORING),
                                                outake.setInnerRotationAction(TeleOp.ROT_OUT_OTLEAVE),
                                                new SleepAction(armRaiseTime),
                                                outake.setMunecaAction(TeleOp.MUNECA_O_FLIPPED),
                                                climber.setTarget(5)
                                        )
                                ),
                                new SleepAction(0.2),
                                outake.setGarraAction(TeleOp.GARRA_ABIERTA_O),
                                new SleepAction(0.3),
                                outake.setRotationAction(TeleOp.BRAZO_OUT_SCORING-0.08),

                                // RETURN FOR ANOTHER SAMPLE WHILE RESETING OUTAKE
                                new ParallelAction(
                                        thirdReturn.build(),
                                        new SequentialAction(
                                                intake.setRotationAction(TeleOp.BRAZO_IN_IDLE),
                                                intake.setGarraAction(TeleOp.GARRA_ABIERTA_I),
                                                intake.setInnerRotationAction(TeleOp.ROT_IN_DOWN),
                                                intake.setTargetAction(350),
                                                outake.setInnerRotationAction(TeleOp.ROT_OUT_OTTAKE),
                                                new SleepAction(0.5),
                                                outake.setMunecaAction(TeleOp.MUNECA_O_HOR),
                                                outake.setRotationAction(TeleOp.BRAZO_OUT_SPECIMEN)
                                        )
                                ),
                                new SleepAction(0.2)
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
//                                                new SleepAction(armRaiseTime),
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
