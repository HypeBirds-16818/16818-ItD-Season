package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Test_Servos")
public class Test_Servos extends LinearOpMode {

    public String active = "None";
    public int in_gar = 0;
    public int in_rot = 0;
    public int in_brazo = 0;
    public int in_mun = 0;
    public int out_brazo = 0;
    public int out_rot = 0;
    public int out_garra = 0;

    public static int INTAKE_TARGET = 0;
    public static int OUTTAKE_TARGET = 0;
    public static double INTAKE_GARRA = 0;
    public static double INTAKE_ROTACION = 0;
    public static double INTAKE_BRAZO = 0;
    public static double OUTAKE_BRAZO = 0;
    public static double OUTAKE_ROTACION = 0;
    public static double OUTAKE_GARRA = 0;
    public static double INTAKE_MUNECA = 0;


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

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

//            drive.setDrivePowers(
//                    new PoseVelocity2d(
//                            new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x
//                    )
//            );

            switch (in_brazo){
                case 1:
                    intake.setBrazo(INTAKE_BRAZO);
                    break;

                default:
                    break;
            }
            switch (in_rot){
                case 1:
                    intake.setRotation(INTAKE_ROTACION);
                    break;

                default:
                    break;
            }
            switch (in_mun){
                case 1:
                    intake.setMuneca(INTAKE_MUNECA);
                    break;

                default:
                    break;
            }
            switch (out_brazo){
                case 1:
                    outake.setBrazo(OUTAKE_BRAZO);
                    break;

                default:
                    break;
            }
            switch (out_rot){
                case 1:
                    outake.setBrazo(OUTAKE_ROTACION);
                    break;

                default:
                    break;
            }

            if(gamepad1.dpad_up){
                if(in_rot == 0){
                    in_rot = 1;
                    active = "INTAKE ROTACION";
                } else {
                    in_rot = 0;
                    active = "NONE";
                }
            }

            if(gamepad1.dpad_down){
                if(in_mun == 0){
                    in_mun = 1;
                    active = "INTAKE MUNECA";
                } else {
                    in_mun = 0;
                    active = "NONE";
                }
            }

            if(gamepad1.dpad_left){
                if(in_brazo == 0){
                    in_brazo = 1;
                    active = "INTAKE BRAZO";
                } else {
                    in_brazo = 0;
                    active = "NONE";
                }
            }

            if(gamepad1.y){
                if(out_rot == 0){
                    out_rot = 1;
                    active = "OUTAKE ROTACION";
                } else {
                    out_rot = 0;
                    active = "NONE";
                }
            }

            if(gamepad1.x){
                if(out_brazo == 0){
                    out_brazo = 1;
                    active = "OUTAKE BRAZO";
                } else {
                    out_brazo = 0;
                    active = "NONE";
                }
            }

            telemetry.addData("current servo: ", active);
            telemetry.update();

            drive.updatePoseEstimate();
//            intake.updatePID();
//            climber.updatePID();

        }
    }
}
