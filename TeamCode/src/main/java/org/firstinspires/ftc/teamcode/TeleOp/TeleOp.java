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
                    // cerrar garra del outake
                    outake.closeGarra();
                    // intake sliders en 0
                    intake.setTarget(SLIDER_I_IN);
                    // servos en posicion inicial
                    // posicion del brazo del outake inicial
                    outake.setBrazo(BRAZO_OUT_ABAJO);

                    if(gamepad1.a){
                        robotState = RobotState.INTAKING;
                    }
                    if(gamepad1.x){
                        robotState = RobotState.SPECIMING;
                    }
                    break;

                case INTAKING:
                    // extender sliders para intakear
                    intake.setTarget(SLIDER_I_OUT);
                    timer.reset();
                    // abrir garra
                    intake.openGarra();

                    if(timer.seconds() > 1){
                        // poner servo viendo para abajo
                        intake.setBrazo(BRAZO_IN_CIENTOCHENTA);
                        intake.setRotation(ROT_IN_NOVENTA);
                    }

                    if(gamepad1.a){
                        robotState = RobotState.CLOSE_INTAKE;
                    }
                    if(gamepad1.b){
                        robotState = RobotState.IDLE;
                    }
                    break;

                case CLOSE_INTAKE:
                    // cerrar garra del intake
                    intake.closeGarra();
                    // WIP - Sensor de tacto ve si agarramos algo y prende un LED

                    if(gamepad1.a){
                        robotState = RobotState.TRANSFER;
                    }
                    if(gamepad1.b){
                        robotState = RobotState.INTAKING;
                    }
                    break;

                case TRANSFER:
                    // retrae slider
                    intake.setTarget(SLIDER_I_IN);
                    timer.reset();

                    if(timer.seconds() > 1){
                        // gira intake
                        intake.setRotation(ROT_IN_CERO);
                        // gira brazo intake
                        intake.setBrazo(BRAZO_IN_CERO);
                    }

                    if(timer.seconds() > 3){
                        // pone outake en posicion
                        outake.setBrazo(BRAZO_OUT_ABAJO);
                        outake.setRotation(ROT_OUT_CERO);
                    }
                    if(timer.seconds() > 4){
                        // cierra outake
                        outake.closeGarra();
                    }
                    if(timer.seconds() > 5){
                        // abre intake
                        intake.openGarra();
                    }


                    if(gamepad2.a){
                        robotState = RobotState.OUTAKING;
                    }
                    break;

                case OUTAKING:
                    // pone la altura del climber en 0
                    climber.setTarget(CLIMBER_ABAJO);
                    // levanza el brazo el outake
                    outake.setBrazo(BRAZO_OUT_MEDIO);
                    timer.reset();

                    if(timer.seconds() > 1){
                        // da la vuelta al outake 270°
                        outake.setRotation(ROT_OUT_CIENTOCHENTA);
                    }

                    if(timer.seconds() > 3){
                        // levanta totalmente el brazo del outake
                        outake.setBrazo(BRAZO_OUT_ARRIBA);
                    }

                    // EN PARALELO
                    // regresa la posicion del intake a la de IDLE

                    if(gamepad2.right_bumper){
                        robotState = RobotState.RAISE1;
                    }
                    if(gamepad2.b){
                        robotState = RobotState.RESET_OUTAKE;
                    }
                    break;

                case RAISE1:
                    // levantar el climber a la altura de la primera canasta
                    climber.setTarget(CLIMBER_F_BASKET);

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
                    climber.setTarget(CLIMBER_S_BASKET);

                    if(gamepad2.left_bumper){
                        robotState = RobotState.RAISE1;
                    }
                    if(gamepad2.a){
                        robotState = RobotState.DROPPING;
                    }
                    break;

                case DROPPING:
                    // abre la garra para dejar caer el sample
                    outake.openGarra();

                    if(gamepad2.b){
                        robotState = RobotState.OUTAKING;
                    }
                    break;

                case SPECIMING:
                    // levantar levemente el climber
                    climber.setTarget(CLIMBER_SLIGHTLY);
                    timer.reset();

                    if(timer.seconds() > 1){
                        // rotar 180° el brazo del outake
                        outake.setBrazo(BRAZO_OUT_ATRAS);
                        // abrir garra
                        outake.openGarra();
                    }

                    if(gamepad1.x){
                        robotState = RobotState.CLOSE_S;
                    }
                    if(gamepad1.b){
                        robotState = RobotState.IDLE;
                    }
                    break;

                case CLOSE_S:
                    // cerrar garra del outake
                    outake.closeGarra();

                    if(gamepad1.x){
                        robotState = RobotState.OUTAKING_S;
                    }
                    if(gamepad1.b){
                        robotState = RobotState.SPECIMING;
                    }
                    break;

                case OUTAKING_S:
                    // levanta un poquito el climber
                    climber.setTarget(CLIMBER_SLIGHTLY);
                    timer.reset();

                    if(timer.seconds() > 1){
                        // rota 180° el outake
                        outake.setRotation(ROT_OUT_CIENTOCHENTA);
                    }

                    if(gamepad2.right_bumper){
                        robotState = RobotState.RAISE_S;
                    }
                    break;

                case RAISE_S:
                    // levanta el climber al upper rung
                    climber.setTarget(CLIMBER_RUNG);

                    if(gamepad2.left_bumper){
                        robotState = RobotState.OUTAKING_S;
                    }
                    if(gamepad2.a){
                        robotState = RobotState.DROPPING_S;
                    }
                    break;

                case DROPPING_S:
                    timer.reset();
                    // baja levemente el slider
                    climber.setTarget(CLIMBER_RUNG-300);
                    // abre la garra

                    if(timer.seconds() > 1){
                        outake.openGarra();
                        // baja el climber a ligeramente arriba de 0
                        climber.setTarget(CLIMBER_SLIGHTLY);
                    }

                    // EN PARALELO
                    // regresa el intake a posicion de IDLE


                    if(gamepad2.a){
                        robotState = RobotState.RESET_OUTAKE;
                    }
                    break;

                case RESET_OUTAKE:
                    // climber ligeramente arriba de 0
                    climber.setTarget(CLIMBER_SLIGHTLY);
                    timer.reset();

                    if(timer.seconds() > 1){
                        // brazo de outtake recto
                        outake.setBrazo(BRAZO_OUT_MEDIO);
                    }
                    if(timer.seconds() > 2){
                        // gira 180° el outake
                        outake.setRotation(ROT_OUT_CERO);
                    }

                    // brazo de outake default
                    if(timer.seconds() > 3){
                        robotState = RobotState.IDLE;
                    }
                    break;
            }



            drive.updatePoseEstimate();
            intake.updatePID();
            climber.updatePID();

        }
    }
}
