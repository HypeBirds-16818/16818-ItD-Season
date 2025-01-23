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
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "HiWorld")
public class HiWorld extends LinearOpMode {
    public static int targetIntake = 0;
    public static int targetOutake = 0;

    public enum RobotState{
        PRE_IDLE,
        IDLE,
        INTAKING,
        HI
    }

    RobotState robotState = RobotState.PRE_IDLE;

    public static double ROT_OUT_CERO = 0.97;
    public static double ROT_OUT_CIENTOCHENTA = 0;
    public static int CLIMBER_ABAJO = -50;
    public static int CLIMBER_SLIGHTLY = -400;
    public static int CLIMBER_F_BASKET = -1000;
    public static int CLIMBER_S_BASKET = -2000;
    public static int CLIMBER_RUNG = -2500;
    public static double BRAZO_OUT_ABAJO = 0.1;
    public static double BRAZO_OUT_MEDIO = 0.3;
    public static double BRAZO_OUT_ARRIBA = 0.7;
    public static double BRAZO_OUT_ATRAS = 1;
    public static double GARRA_ABIERTA_I = 0.6;
    public static double GARRA_CERRADA_I = 0.99;

    public static double GARRA_ABIERTA_O = 0;
    public static double GARRA_CERRADA_O = 0.19;
    public static int SLIDER_I_IN = 0;
    public static int SLIDER_I_OUT = 500;
    public static double ROT_IN_CERO = 1;
    public static double ROT_IN_NOVENTA = 0.645;
    public static double ROT_IN_CIENTOCHENTA = 1;
    public static double BRAZO_IN_CERO = 1;
    public static double BRAZO_IN_NOVENTA = 0.6;
    public static double BRAZO_IN_CIENTOCHENTA = 0.17;
    public static double MUNECA_VER = 0.65;
    public static double MUNECA_HOR = 0.97;

    public boolean flag = false;
    public boolean flag2 = false;

    public double ir = ROT_IN_CERO;
    public double im = MUNECA_VER;
    public double ib = BRAZO_IN_CERO;
    public double or = ROT_OUT_CERO;
    public double ob = BRAZO_OUT_MEDIO;
    public double ig = GARRA_ABIERTA_I;
    public double og = GARRA_ABIERTA_O;

    public String state = "None";


    @Override
    public void runOpMode() throws InterruptedException {
        Intake intake = new Intake(hardwareMap);
        Outtake outake = new Outtake(hardwareMap);
        Climber climber = new Climber(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0, 0));
        ElapsedTime timer = new ElapsedTime();

        waitForStart();
        intake.init(hardwareMap);
//        outake.init();
        climber.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

//            drive.setDrivePowers(
//                    new PoseVelocity2d(
//                            new Vector2d(-gamepad1.left_stick_y * 0.8, -gamepad1.left_stick_x), -gamepad1.right_stick_x * 0.5
//                    )
//            );

            switch(robotState){
                case PRE_IDLE:
                    state = "PRE_IDLE";
                    ob =  BRAZO_OUT_MEDIO;
                    or = ROT_OUT_CERO;
                    ib = BRAZO_IN_CERO;
                    ir = ROT_IN_CERO;
                    im = MUNECA_VER;
                    ig = GARRA_ABIERTA_I;
                    og = GARRA_ABIERTA_O;
                    targetOutake = 0;
                    targetIntake = -50;
                    timer.reset();
                    robotState = RobotState.IDLE;
                case IDLE:
                    if(timer.seconds() < 1){
                        state = "IDLE";
                        // posicion del brazo del outake inicial
                        ob = (BRAZO_OUT_MEDIO);
                        // cerrar garra del outake
                        og = GARRA_CERRADA_O;
                        // servos en posicion inicial
                        ib = BRAZO_IN_CERO;
                        ir = ROT_IN_CERO;
                    }

                    if(timer.seconds() > 1){
                        // intake sliders en 0
                        targetIntake = 0;
                        targetOutake = -50;
                        if(gamepad1.a){
                            robotState = RobotState.INTAKING;
                            timer.reset();
                        }

                    }

                    break;

                case INTAKING:
                    state = "INTAKING";
                    // abrir garra
                    ig = GARRA_ABIERTA_I;
                    og = GARRA_ABIERTA_O;

                    if((gamepad1.a && flag)){
                        robotState = RobotState.HI;
                    }

                    if(gamepad1.dpad_up){
                        targetIntake = SLIDER_I_OUT;
                        ib = (BRAZO_IN_NOVENTA);
                        im = MUNECA_VER;
                        flag2 = true;
                        timer.reset();
                    }

                    if(flag2 && timer.seconds() > 1){
                        ir = (ROT_IN_NOVENTA);
                        ib = BRAZO_IN_NOVENTA;
                        flag2 = false;
                        flag = true;
                    }
                    break;

                case HI:
                    targetOutake = CLIMBER_SLIGHTLY;

                    if(timer.milliseconds() < 800){
                        outake.setBrazo(BRAZO_OUT_ARRIBA);
                        intake.setMuneca(MUNECA_HOR);
                    }
                    if(timer.milliseconds() > 800 && timer.milliseconds() < 1600){
                        outake.setBrazo(BRAZO_OUT_MEDIO);
                        intake.setMuneca(MUNECA_VER);
                    }

                    if(timer.milliseconds() > 1600){
                        timer.reset();
                    }



            }
            drive.updatePoseEstimate();
            intake.updatePID(targetIntake);
            climber.updatePID(targetOutake);
            runServos(intake, outake, ib, im, ir, ob, or, og, ig);

            telemetry.addData("current state: ", state);
            telemetry.update();

        }
    }

    public void runServos(Intake intake, Outtake outake, double IN_BRAZO, double IN_MUNECA, double IN_ROTACION, double OUT_BRAZO, double OUT_ROTACION, double OUT_GARRA, double IN_GARRA){
        intake.setBrazo(IN_BRAZO);
        intake.setRotation(IN_ROTACION);
        intake.setMuneca(IN_MUNECA);
        outake.setBrazo(OUT_BRAZO);
        outake.setRotation(OUT_ROTACION);
        intake.setGarra(IN_GARRA);
        outake.setGarra(OUT_GARRA);
    }
}
