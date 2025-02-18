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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.Climber;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Outtake;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOp;

@Autonomous(name = "uwu rizzler")
public class AutoSpecimens extends LinearOpMode {

    public double armRaiseTime = 0.8;

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
                .strafeTo(new Vector2d(1, -31.73));


        TrajectoryActionBuilder positionToMove = driveToFirstSample.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(1,-40.73))
                .strafeToLinearHeading(new Vector2d(29, -40.73), Math.toRadians(50));

        TrajectoryActionBuilder moveFirst = positionToMove.endTrajectory().fresh()
                .turnTo(Math.toRadians(-40));

        TrajectoryActionBuilder planSecond = moveFirst.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(40,-40), Math.toRadians(50));

        TrajectoryActionBuilder moveSecond = planSecond.endTrajectory().fresh()
                .turnTo(Math.toRadians(-40));

        TrajectoryActionBuilder planThird = moveSecond.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(52,-40), Math.toRadians(50));

        TrajectoryActionBuilder moveThird = planThird.endTrajectory().fresh()
                .turn(Math.toRadians(-40));

        TrajectoryActionBuilder setForSecondSample = moveThird.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(40, -64.90), Math.toRadians(270));

        TrajectoryActionBuilder driveToSecondSample = setForSecondSample.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(5, -31.7));

        TrajectoryActionBuilder firstReturn = driveToSecondSample.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(5,-40))
                .strafeToConstantHeading(new Vector2d(36.60, -64));

        TrajectoryActionBuilder driveToThirdSample = firstReturn.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(3, -31.7));

        TrajectoryActionBuilder secondReturn = driveToThirdSample.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(3,-40))
                .strafeToConstantHeading(new Vector2d(36.60, -64));

        TrajectoryActionBuilder driveToFourthSample = secondReturn.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(5, -31.7));

        TrajectoryActionBuilder thirdReturn = driveToFourthSample.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(36.60, -64));

        TrajectoryActionBuilder driveToFifthSample = thirdReturn.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(3, -31.7));

        TrajectoryActionBuilder fourthReturn = driveToFifthSample.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(36.60, -64));



        waitForStart();


        Actions.runBlocking(new SequentialAction(climber.setTarget(30), intake.setTargetAction(0),
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

                                // INIT OUTAKE TO MOVE SAMPLES IN FLOOR
                                new ParallelAction(
                                        positionToMove.build(),
                                        new SequentialAction(
                                                new SleepAction(1),
                                                outake.setInnerRotationAction(TeleOp.ROT_OUT_OTTAKE),
                                                outake.setMunecaAction(TeleOp.MUNECA_O_HOR),
                                                outake.setRotationAction(TeleOp.BRAZO_OUT_SPECIMEN)
                                        )
                                ),
                                new SleepAction(0.2),
                                intake.setTargetAction(250),
                                new SleepAction(0.5),
                                moveFirst.build(),
                                planSecond.build(),
                                moveSecond.build(),
                                planThird.build(),
                                moveThird.build(),
                                intake.setTargetAction(-100),
                                intake.setModeAction(DcMotor.RunMode.STOP_AND_RESET_ENCODER),
                                intake.setModeAction(DcMotor.RunMode.RUN_USING_ENCODER),
                                setForSecondSample.build(),
                                new SleepAction(10),




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

                                // RETURN FOR ANOTHER SAMPLE WHILE RESETING OUTAKE
                                new ParallelAction(
                                        firstReturn.build(),
                                        new SequentialAction(
                                                outake.setInnerRotationAction(TeleOp.ROT_OUT_OTTAKE),
                                                new SleepAction(0.5),
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

                                // RETURN FOR ANOTHER SAMPLE WHILE RESETING OUTAKE
                                new ParallelAction(
                                        secondReturn.build(),
                                        new SequentialAction(
                                                outake.setInnerRotationAction(TeleOp.ROT_OUT_OTTAKE),
                                                new SleepAction(0.5),
                                                outake.setMunecaAction(TeleOp.MUNECA_O_HOR),
                                                outake.setRotationAction(TeleOp.BRAZO_OUT_SPECIMEN)
                                        )
                                )
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
