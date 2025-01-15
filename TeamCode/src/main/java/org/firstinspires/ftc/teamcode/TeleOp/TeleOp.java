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
public class TeleOp extends LinearOpMode {
    public static int targetIntake = 0;
    public static int targetOutake = 0;

    public enum RobotState{
        IDLE,
        INTAKING,
        CLOSE_INTAKE,
        PRE_TRANSFER,
        DROP,
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

    RobotState robotState = RobotState.IDLE;

    public static double ROT_OUT_CERO = 0;
    public static double ROT_OUT_CIENTOCHENTA = 1;
    public static int CLIMBER_ABAJO = 0;
    public static int CLIMBER_SLIGHTLY = 400;
    public static int CLIMBER_F_BASKET = 1000;
    public static int CLIMBER_S_BASKET = 2000;
    public static int CLIMBER_RUNG = 2500;
    public static double BRAZO_OUT_ABAJO = 0;
    public static double BRAZO_OUT_MEDIO = 0.25;
    public static double BRAZO_OUT_ARRIBA = 0.5;
    public static double BRAZO_OUT_ATRAS = 1;
    public static double GARRA_ABIERTA = 1;
    public static double GARRA_CERRADA = 0;
    public static int SLIDER_I_IN = 0;
    public static int SLIDER_I_OUT = 500;
    public static double ROT_IN_CERO = 0;
    public static double ROT_IN_NOVENTA = 0.5;
    public static double ROT_IN_CIENTOCHENTA = 1;
    public static double BRAZO_IN_CERO = 0;
    public static double BRAZO_IN_NOVENTA = 0.5;
    public static double BRAZO_IN_CIENTOCHENTA = 1;

    public boolean flag = false;
    public boolean flag2 = false;
    
    public double ir = 0;
    public double im = 0;
    public double ib = 0;
    public double or = 0;
    public double ob = 0;
    public double ig = 0;
    public double og = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        Intake intake = new Intake(hardwareMap);
        Outtake outake = new Outtake(hardwareMap);
        Climber climber = new Climber(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0, 0));
        ElapsedTime timer = new ElapsedTime();

        waitForStart();
        intake.init();
        outake.init();
        climber.init();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(-gamepad1.left_stick_y * 0.8, -gamepad1.left_stick_x), -gamepad1.right_stick_x * 0.5
                    )
            );

            switch(robotState){
                case IDLE:
                    if(timer.seconds() < 1){
                        // posicion del brazo del outake inicial
                        ob = (BRAZO_OUT_MEDIO);
                        // cerrar garra del outake
                        og = GARRA_CERRADA;
                        // servos en posicion inicial
                        ib = BRAZO_IN_CERO;
                        ir = ROT_IN_CERO;
                    }

                    if(timer.seconds() > 1){
                        // intake sliders en 0
                        targetIntake = 0;
                        if(gamepad1.a){
                            robotState = RobotState.INTAKING;
                        }
                        if(gamepad1.x){
                            robotState = RobotState.SPECIMING;
                            timer.reset();
                        }
                    }

                    break;

                case INTAKING:

                    // abrir garra
                    ig = GARRA_ABIERTA;

                    if((gamepad1.a && flag)){
                        robotState = RobotState.CLOSE_INTAKE;
                    }
                    if((gamepad1.b && flag)){
                        robotState = RobotState.IDLE;
                        timer.reset();
                    }
                    if(gamepad1.dpad_up){
                        targetIntake = SLIDER_I_OUT;
                        ib = (BRAZO_IN_NOVENTA);
                        flag2 = true;
                        timer.reset();
                    }
                    if(gamepad1.dpad_down){
                        targetIntake = SLIDER_I_IN;
                        ib = (BRAZO_IN_CIENTOCHENTA);
                        flag2 = true;
                        timer.reset();

                    }
                    if(flag2 && timer.seconds() > 1){
                        ir = (ROT_IN_NOVENTA);
                        ib = BRAZO_IN_CIENTOCHENTA;
                        flag2 = false;
                        flag = true;
                    }
                    break;

                case CLOSE_INTAKE:
                    // cerrar garra del intake
                    ig = GARRA_CERRADA;
                    // WIP - Sensor de tacto ve si agarramos algo y prende un LED

                    if(gamepad1.a){
                        robotState = RobotState.PRE_TRANSFER;
                    }
                    if(gamepad1.b){
                        robotState = RobotState.INTAKING;
                    }
                    break;

                case PRE_TRANSFER:

                    ib = (BRAZO_IN_NOVENTA);

                    // retrae slider
                    targetIntake = SLIDER_I_IN;

                    if(gamepad1.a){
                        robotState = RobotState.TRANSFER;
                        timer.reset();
                    }
                    if(gamepad1.b){
                        robotState = RobotState.DROP;
                        timer.reset();
                    }
                    break;

                case DROP:
                    if(timer.seconds() < 1){
                        ib = BRAZO_IN_CIENTOCHENTA;
                    }
                    if(timer.seconds() > 1 && timer.seconds() < 2){
                        ig = GARRA_ABIERTA;
                    }
                    if(timer.seconds() > 2 && timer.seconds() < 3){
                        ib = BRAZO_IN_NOVENTA;
                    }
                    if(timer.seconds() > 3){
                        robotState = RobotState.IDLE;
                        timer.reset();
                    }

                case TRANSFER:
                    if(timer.seconds() < 1){
                        // gira intake
                        ir = (ROT_IN_CERO);
                        // gira brazo intake
                        ob = (BRAZO_OUT_MEDIO);
                    }

                    if(timer.seconds() > 1 && timer.seconds() < 2){
                        // pone outake en posicion
                        ib = (BRAZO_IN_CERO);
                    }
                    if(timer.seconds() > 2 && timer.seconds() < 3){
                        ob = (BRAZO_OUT_ABAJO);
                    }
                    if(timer.seconds() > 3 && timer.seconds() < 4){
                        // abre intake
                        og = GARRA_CERRADA;
                    }
                    if(timer.seconds() > 4){
                        ig = GARRA_ABIERTA;

                        if(gamepad2.a){
                            robotState = RobotState.OUTAKING;
                            timer.reset();
                        }
                    }
                    break;

                case OUTAKING:
                    if(timer.seconds() < 1){
                        // pone la altura del climber en 0
                        targetOutake = CLIMBER_ABAJO;
                        // levanza el brazo el outake
                        ob = (BRAZO_OUT_MEDIO);
                    }



                    if(timer.seconds() > 1 && timer.seconds() < 3){
                        // da la vuelta al outake 270°
                        or = (ROT_OUT_CIENTOCHENTA);
                    }

                    if(timer.seconds() > 3){
                        // levanta totalmente el brazo del outake
                        ob = (BRAZO_OUT_ARRIBA);
                        targetOutake = CLIMBER_ABAJO;

                        if(gamepad2.right_bumper){
                            robotState = RobotState.RAISE1;
                        }
                        if(gamepad2.b){
                            robotState = RobotState.RESET_OUTAKE;
                            timer.reset();
                        }
                    }

                    // EN PARALELO
                    // regresa la posicion del intake a la de IDLE


                    break;

                case RAISE1:
                    // levantar el climber a la altura de la primera canasta
                    targetOutake = CLIMBER_F_BASKET;

                    if(gamepad2.left_bumper){
                        robotState = RobotState.OUTAKING;
                    }
                    if(gamepad2.right_bumper){
                        robotState = RobotState.RAISE2;
                    }
                    if(gamepad2.a){
                        robotState = RobotState.DROPPING;
                    }
                    break;

                case RAISE2:
                    // levantar el climber a la altura de la segunda canasta
                    targetOutake = CLIMBER_S_BASKET;

                    if(gamepad2.left_bumper){
                        robotState = RobotState.RAISE1;
                    }
                    if(gamepad2.a){
                        robotState = RobotState.DROPPING;
                    }
                    break;

                case DROPPING:
                    // abre la garra para dejar caer el sample
                    og = GARRA_ABIERTA;

                    if(gamepad2.b){
                        robotState = RobotState.OUTAKING;
                    }
                    break;

                case SPECIMING:
                    if(timer.seconds() < 1){
                        // levantar levemente el climber
                        targetOutake = CLIMBER_SLIGHTLY;
                    }

                    if(timer.seconds() > 1){
                        // rotar 180° el brazo del outake
                        ob = (BRAZO_OUT_ATRAS);
                        // abrir garra
                        og = GARRA_ABIERTA;
                    }

                    if(gamepad1.x){
                        robotState = RobotState.CLOSE_S;
                    }
                    if(gamepad1.b){
                        robotState = RobotState.IDLE;
                        timer.reset();
                    }
                    break;

                case CLOSE_S:
                    // cerrar garra del outake
                    og = GARRA_CERRADA;

                    if(gamepad1.x){
                        robotState = RobotState.OUTAKING_S;
                        
                    }
                    if(gamepad1.b){
                        robotState = RobotState.SPECIMING;
                    }
                    break;

                case OUTAKING_S:
                    // levanta un poquito el climber
                    targetOutake = CLIMBER_SLIGHTLY;
                
                    // rota 180° el outake
                    or = (ROT_OUT_CIENTOCHENTA);
                    
                    if(gamepad2.right_bumper){
                        robotState = RobotState.RAISE_S;
                    }
                    break;

                case RAISE_S:
                    // levanta el climber al upper rung
                    targetOutake = CLIMBER_RUNG;

                    if(gamepad2.left_bumper){
                        robotState = RobotState.OUTAKING_S;
                    }
                    if(gamepad2.a){
                        robotState = RobotState.DROPPING_S;
                        timer.reset();
                    }
                    break;

                case DROPPING_S:
                    if(timer.seconds() < 1){
                        // baja levemente el slider
                        targetOutake = CLIMBER_RUNG-300;
                    }

                    if(timer.seconds() > 1){
                        // abre la garra
                        og = GARRA_ABIERTA;
                        // baja el climber a ligeramente arriba de 0
                        targetOutake = CLIMBER_SLIGHTLY;
                    }

                    // EN PARALELO
                    // regresa el intake a posicion de IDLE

                    if(gamepad2.a){
                        robotState = RobotState.RESET_OUTAKE;
                        timer.reset();
                    }
                    break;

                case RESET_OUTAKE:
                    if(timer.seconds() < 1){
                        // climber ligeramente arriba de 0
                        targetOutake = CLIMBER_SLIGHTLY;
                    }
                    if(timer.seconds() > 1 && timer.seconds() < 2){
                        // brazo de outtake recto
                        ob = (BRAZO_OUT_MEDIO);
                    }
                    if(timer.seconds() > 2 && timer.seconds() < 3){
                        // gira 180° el outake
                        or = (ROT_OUT_CERO);
                    }

                    // brazo de outake default
                    if(timer.seconds() > 3){
                        robotState = RobotState.IDLE;
                        timer.reset();
                    }
                    break;
            }
            drive.updatePoseEstimate();
            intake.updatePID(targetIntake);
            climber.updatePID(targetOutake);
            runServos(intake, outake, ib, im, ir, ob, or, og, ig);

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
