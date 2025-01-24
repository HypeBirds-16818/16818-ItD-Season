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

import org.firstinspires.ftc.teamcode.mechanisms.Climber;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Outtake;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOp;

@Autonomous
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(8.74, -62.90, Math.toRadians(270.00));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Outtake outake = new Outtake(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Climber climber = new Climber(hardwareMap);

        climber.init(hardwareMap);
        intake.init(hardwareMap);

        TrajectoryActionBuilder driveToFirstSample = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(1.95, -33.73));


        TrajectoryActionBuilder moveAllSamples = driveToFirstSample.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(31.70, -62.90), Math.toRadians(270.00))
                .strafeToConstantHeading(new Vector2d(36.76, -13.65))
                .strafeToConstantHeading(new Vector2d(48.60, -13.94))
                .strafeToConstantHeading(new Vector2d(48.75, -60.45))
                .strafeToConstantHeading(new Vector2d(48.60, -9.17))
                .strafeToConstantHeading(new Vector2d(57.99, -14.23))
                .strafeToConstantHeading(new Vector2d(57.99, -60.16))
                .strafeToConstantHeading(new Vector2d(57.99, -9.17))
                .strafeToConstantHeading(new Vector2d(62.47, -9.32))
                .strafeToConstantHeading(new Vector2d(62.32, -54.81))
                .waitSeconds(2)
                .strafeToConstantHeading(new Vector2d(36.32, -62.90));

        TrajectoryActionBuilder driveToSecondSample = moveAllSamples.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(3.83, -33.29));

        TrajectoryActionBuilder firstReturn = driveToSecondSample.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(35.60, -61.17), Math.toRadians(270.00));

        TrajectoryActionBuilder driveToThirdSample = firstReturn.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(1.83, -33.29));

        TrajectoryActionBuilder secondReturn = driveToThirdSample.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(35.60, -61.17), Math.toRadians(270.00));

        TrajectoryActionBuilder driveToFourthSample = secondReturn.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-0.83, -33.29));

        TrajectoryActionBuilder thirdReturn = driveToFourthSample.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(35.60, -61.17), Math.toRadians(270.00));

        TrajectoryActionBuilder driveToFifthSample = thirdReturn.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-2.83, -33.29));

        TrajectoryActionBuilder fourthReturn = driveToFifthSample.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(35.60, -61.17), Math.toRadians(270.00));

        waitForStart();


        Actions.runBlocking(new SequentialAction(climber.setTarget(TeleOp.CLIMBER_SLIGHTLY),
                new ParallelAction(
                        climber.updatePIDAction(),
                        new SequentialAction(
                        //INIT
                                outake.setRotationAction(TeleOp.BRAZO_OUT_SPECIMEN),
                                outake.setMunecaAction(TeleOp.MUNECA_O_HOR),
                                outake.setGarraAction(TeleOp.GARRA_CERRADA_O),
                                outake.setInnerRotationAction(TeleOp.ROT_OUT_OTTAKE),

                                intake.setSlidersAction(TeleOp.SLIDER_I_IN),
                                intake.setRotationAction(TeleOp.BRAZO_IN_IDLE),
                                intake.setGarraAction(TeleOp.GARRA_ABIERTA_I),
                                intake.setInnerRotationAction(TeleOp.ROT_IN_DOWN),
                                intake.setMunecaAction(TeleOp.MUNECA_I_HOR),

                                // SPECIMEN OUTAKE
                                new SleepAction(0.2),
                                climber.setTarget(TeleOp.CLIMBER_SLIGHTLY-200),
                                outake.setRotationAction(TeleOp.BRAZO_OUT_SCORING),
                                outake.setInnerRotationAction(TeleOp.ROT_OUT_OTLEAVE),
                                new SleepAction(0.5),
                                outake.setMunecaAction(TeleOp.MUNECA_O_FLIPPED),
                                new SleepAction(0.2),
                                climber.setTarget(TeleOp.CLIMBER_SLIGHTLY),

                                // MOVE TO FIRST SAMPLE
                                driveToFirstSample.build(),

                                new SleepAction(1),
                                outake.setGarraAction(TeleOp.GARRA_ABIERTA_O),

                                // INIT OUTAKE TO MOVE SAMPLES IN FLOOR
                                new ParallelAction(
                                        moveAllSamples.build(),
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
                                new SleepAction(0.2),
                                climber.setTarget(TeleOp.CLIMBER_SLIGHTLY-200),

                                // MOVE TO SECOND SAMPLE WHILE MOVING OUTAKE
                                new ParallelAction(
                                        driveToSecondSample.build(),
                                        new SequentialAction(
                                                outake.setRotationAction(TeleOp.BRAZO_OUT_SCORING),
                                                outake.setInnerRotationAction(TeleOp.ROT_OUT_OTLEAVE),
                                                new SleepAction(0.5),
                                                outake.setMunecaAction(TeleOp.MUNECA_O_FLIPPED),
                                                new SleepAction(0.2),
                                                climber.setTarget(TeleOp.CLIMBER_SLIGHTLY)
                                        )
                                ),
                                new SleepAction(1),
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
                                new SleepAction(0.2),
                                climber.setTarget(TeleOp.CLIMBER_SLIGHTLY-200),

                                // MOVE TO THIRD SAMPLE WHILE MOVING OUTAKE
                                new ParallelAction(
                                        driveToThirdSample.build(),
                                        new SequentialAction(
                                                outake.setRotationAction(TeleOp.BRAZO_OUT_SCORING),
                                                outake.setInnerRotationAction(TeleOp.ROT_OUT_OTLEAVE),
                                                new SleepAction(0.5),
                                                outake.setMunecaAction(TeleOp.MUNECA_O_FLIPPED),
                                                new SleepAction(0.2),
                                                climber.setTarget(TeleOp.CLIMBER_SLIGHTLY)
                                        )
                                ),
                                new SleepAction(1),
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
                                ),
                                new SleepAction(0.2),
                                // CLOSE CLAW
                                outake.setGarraAction(TeleOp.GARRA_CERRADA_O),
                                new SleepAction(0.2),
                                climber.setTarget(TeleOp.CLIMBER_SLIGHTLY-200),

                                // MOVE TO FOURTH SAMPLE WHILE MOVING OUTAKE
                                new ParallelAction(
                                        driveToFourthSample.build(),
                                        new SequentialAction(
                                                outake.setRotationAction(TeleOp.BRAZO_OUT_SCORING),
                                                outake.setInnerRotationAction(TeleOp.ROT_OUT_OTLEAVE),
                                                new SleepAction(0.5),
                                                outake.setMunecaAction(TeleOp.MUNECA_O_FLIPPED),
                                                new SleepAction(0.2),
                                                climber.setTarget(TeleOp.CLIMBER_SLIGHTLY)
                                        )
                                ),
                                new SleepAction(1),
                                outake.setGarraAction(TeleOp.GARRA_ABIERTA_O),

                                // RETURN FOR ANOTHER SAMPLE WHILE RESETING OUTAKE
                                new ParallelAction(
                                        thirdReturn.build(),
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
                                new SleepAction(0.2),
                                climber.setTarget(TeleOp.CLIMBER_SLIGHTLY-200),

                                // MOVE TO FIFTH SAMPLE WHILE MOVING OUTAKE
                                new ParallelAction(
                                        driveToFifthSample.build(),
                                        new SequentialAction(
                                                outake.setRotationAction(TeleOp.BRAZO_OUT_SCORING),
                                                outake.setInnerRotationAction(TeleOp.ROT_OUT_OTLEAVE),
                                                new SleepAction(0.5),
                                                outake.setMunecaAction(TeleOp.MUNECA_O_FLIPPED),
                                                new SleepAction(0.2),
                                                climber.setTarget(TeleOp.CLIMBER_SLIGHTLY)
                                        )
                                ),
                                new SleepAction(1),
                                outake.setGarraAction(TeleOp.GARRA_ABIERTA_O),

                                // RETURN FOR ANOTHER SAMPLE WHILE RESETING OUTAKE
                                new ParallelAction(
                                        fourthReturn.build(),
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
                                new SleepAction(0.2),
                                climber.setTarget(TeleOp.CLIMBER_SLIGHTLY-200)
                ))));


    }
}
