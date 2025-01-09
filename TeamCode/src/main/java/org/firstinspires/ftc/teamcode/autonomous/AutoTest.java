package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(9.6, -62.61, 90);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(3.68, -33.73), Math.toRadians(90.00))
                .strafeTo(new Vector2d(34.30, -36.04))
                .splineToConstantHeading(new Vector2d(40.66, -8.45), Math.toRadians(90.00))
                .strafeTo(new Vector2d(46.87, -58.86))
                .splineToConstantHeading(new Vector2d(50.05, -8.74), Math.toRadians(90.00))
                .strafeTo(new Vector2d(56.55, -58.71))
                .splineToConstantHeading(new Vector2d(59.00, -8.59), Math.toRadians(90.00))
                .strafeTo(new Vector2d(63.05, -8.59))
                .strafeTo(new Vector2d(62.90, -58.57));

        Action trajectoryActionChosen;
        trajectoryActionChosen = tab1.build();

    }
}
