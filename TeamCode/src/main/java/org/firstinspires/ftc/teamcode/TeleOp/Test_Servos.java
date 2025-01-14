package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Climber;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Outtake;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpToluca")
public class Test_Servos extends LinearOpMode {
    public static int targetIntake = 0;
    public static int targetOutake = 0;

    public enum RobotState{
        IDLE,
        INTAKING,
        CLOSE_INTAKE,
        TRANSFER,
        OUTAKING,
        RAISE1,
        RAISE2,
        DROPPING,
        SPECIMING,
        CLOSE_S,
        OUTAKING_S,
        RAISE_S,
        DROPPING_S,
        RESET_OUTAKE
    }


    public static int SLIDER_I_OUT = 500;
    public static double ROT_IN_CERO = 0;
    public static double ROT_IN_NOVENTA = 0.5;
    public static double ROT_IN_CIENTOCHENTA = 1;
    public static double BRAZO_IN_CERO = 0;
    public static double BRAZO_IN_NOVENTA = 0.5;
    public static double BRAZO_IN_CIENTOCHENTA = 1;


    @Override
    public void runOpMode() throws InterruptedException {
        Intake intake = new Intake(hardwareMap);
        Outtake outake = new Outtake(hardwareMap);
        Climber climber = new Climber(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0, 0));
        ElapsedTime timer = new ElapsedTime();

        waitForStart();
//        intake.init();
//        outake.init();
//        climber.init();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

//            drive.setDrivePowers(
//                    new PoseVelocity2d(
//                            new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x
//                    )
//            );



            drive.updatePoseEstimate();
            intake.updatePID();
            climber.updatePID();

        }
    }
}
