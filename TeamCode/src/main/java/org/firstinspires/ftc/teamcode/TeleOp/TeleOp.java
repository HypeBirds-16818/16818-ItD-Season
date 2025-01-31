package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.mechanisms.Climber;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Outtake;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpToluca")
public class TeleOp extends LinearOpMode {
    public static int targetOutake = 0;

    private enum RobotState{
        PRE_IDLE,
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
        PRE_OUTAKING_S,
        OUTAKING_S,
        RAISE_S,
        DROPPING_S,
        RESET_OUTAKE
    }

    RobotState robotState = RobotState.PRE_IDLE;

    // ROTACION INTERNA OUTAKE (THEORIZED)
    public static double ROT_OUT_START = 0.22; // 0° REF
    public static double ROT_OUT_OTTAKE = 0.22; // 45°
    public static double ROT_OUT_TRANSFER = 0.55; // 135°
    public static double ROT_OUT_OTLEAVE = 0.55; // ?
    public static double ROT_OUT_PRETAKE = 0.75; // 180°

    // SLIDER CLIMBER
    public static int CLIMBER_ABAJO = 50;
    public static int CLIMBER_SLIGHTLY = 50; // FOR SPECIMEN INTAKEING
    public static int CLIMBER_F_BASKET = 1000;
    public static int CLIMBER_S_BASKET = 3100;
    public static int CLIMBER_RUNG = 2100;

    // BRAZO OUTAKE (THEORIZED)
    public static double BRAZO_OUT_IDLE = 0.95; // ~30° 0 REF
    public static double BRAZO_OUT_SPECIMEN = 1; // ~35° 5
    public static double BRAZO_OUT_PREINTAKE = 0.26; // ~95° 65
    public static double BRAZO_OUT_TRANSFER = 0.9; // ~85° 55
    public static double BRAZO_OUT_SCORING = 0.21; // ~225° 195

    // GARRAS
    public static double GARRA_ABIERTA_I = 0.6;
    public static double GARRA_CERRADA_I = 0.99;
    public static double GARRA_ABIERTA_O = 0.9;
    public static double GARRA_CERRADA_O = 0.54;

    // SLIDER INTAKE (THEORIZED)
    public static double SLIDER_I_IN = 0;
    public static double SLIDER_I_QUARTER = 0.0875;
    public static double SLIDER_I_HALF = 0.175;
    public static double SLIDER_I_LASTQUARTER = 0.2625;
    public static double SLIDER_I_OUT = 0.35;

    // ROTACION INTERNA INTAKE (THEORIZED)
    public static double ROT_IN_DOWN = 0.15; // 0°
    public static double ROT_IN_TRANSFER = 0.19; // ~10°
    public static double ROT_IN_PREINTAKE = 0.75; // 180°

    // BRAZO INTAKE (THEORIZED)
    public static double BRAZO_IN_IDLE = 0; // 270° 0
    public static double BRAZO_IN_TRANSFER = 0.45; // ~135° 135
    public static double BRAZO_IN_PREINTAKE = 0.42; // ~120° 150
    public static double BRAZO_IN_INTAKING = 0.6; // ~90° 180

    // MUÑECAS (OFFSETS < 0.15) (THEORIZED)
    public static double MUNECA_O_VER = 0;
    public static double MUNECA_O_HOR = 0.18;
    public static double MUNECA_O_FLIPPED = -0.18;
    public static double MUNECA_I_VER = 0;
    public static double MUNECA_I_HOR = 0;

    // ETC
    public static double velocity = 0.5;
    public static double velocity_rot = 0.5;
    public static double velocity_lat = 0.5;

    public boolean flag = false;
    public boolean flag2 = false;
    private boolean open = false;
    private boolean openS = false;
    public boolean nuke = false;
    private double offsetMuneca = 0;

    public String state = "None";


    @Override
    public void runOpMode() throws InterruptedException {
        Intake intake = new Intake(hardwareMap);
        Outtake outake = new Outtake(hardwareMap);
        Climber climber = new Climber(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0, 0));
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime cd = new ElapsedTime();

        waitForStart();
        intake.init(hardwareMap);
        outake.init(hardwareMap);
        climber.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            double heading = drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(
                    Math.cos(-heading)*-gamepad1.left_stick_y * velocity - Math.sin(-heading)*-gamepad1.left_stick_x * velocity_lat,
                    Math.sin(-heading)*-gamepad1.left_stick_y * velocity + Math.cos(-heading)*-gamepad1.left_stick_x * velocity_lat
            ),
                    -gamepad1.right_stick_x
            ));

//            drive.setDrivePowers(
//                    new PoseVelocity2d(
//                            new Vector2d(-gamepad1.left_stick_y * velocity, -gamepad1.left_stick_x * velocity_lat), -gamepad1.right_stick_x * velocity_rot
//                    )
//            );

            switch(robotState){
                case PRE_IDLE:
                    state = "PRE_IDLE";
                    outake.setRotation(BRAZO_OUT_IDLE);
                    //outake.setMuneca(MUNECA_O_HOR);
                    outake.setGarra(GARRA_ABIERTA_O);
                    outake.setInnerRotation(ROT_OUT_START);

                    intake.setSliders(SLIDER_I_IN);
                    intake.setRotation(BRAZO_IN_IDLE);
                    intake.setGarra(GARRA_ABIERTA_I);
                    intake.setInnerRotation(ROT_IN_DOWN);
                    intake.setMuneca(MUNECA_I_HOR);

                    targetOutake = 50;
                    timer.reset();
                    robotState = RobotState.IDLE;
                    break;

                case IDLE:
                    if(timer.milliseconds() < 100){
                        gamepad1.rumble(200);
                        outake.setMuneca(MUNECA_O_HOR);
                        intake.setMuneca(MUNECA_I_HOR);
                    }
                    if(timer.seconds() < 1){
                        flag = false;
                        flag2 = false;
                        state = "IDLE";

                        // servos de outake en posicion inicial
                        outake.setRotation(BRAZO_OUT_IDLE);
                        outake.setGarra(GARRA_ABIERTA_O);
                        outake.setInnerRotation(ROT_OUT_START);

                        // servos de intake en posicion inicial
                        intake.setSliders(SLIDER_I_IN);
                        intake.setRotation(BRAZO_IN_IDLE);
                        intake.setGarra(GARRA_ABIERTA_I);
                        intake.setInnerRotation(ROT_IN_DOWN);


                    }

                    if(timer.seconds() > 1){
                        targetOutake = 50;
                        if(gamepad1.a){
                            robotState = RobotState.INTAKING;
                            timer.reset();
                        }
                        if(gamepad1.x){
                            robotState = RobotState.SPECIMING;
                            timer.reset();
                        }
                    }

                    break;

                case INTAKING:
                    state = "INTAKING";
                    if(timer.milliseconds() < 300){
                        intake.setMuneca(MUNECA_I_HOR);
                    }
                    if(timer.milliseconds() < 500 && timer.milliseconds() > 50){
                        outake.setRotation(BRAZO_OUT_PREINTAKE);
                        outake.setInnerRotationAction(ROT_OUT_PRETAKE);
                        outake.setGarra(GARRA_ABIERTA_O);
                    }
                    if(timer.milliseconds() > 500 && timer.milliseconds() < 1000){
                        intake.setRotation(BRAZO_IN_PREINTAKE);
                        intake.setGarra(GARRA_ABIERTA_I);
                        intake.setInnerRotation(ROT_IN_PREINTAKE);

                        flag = true;
                    }

                    if(gamepad1.dpad_up){
                        intake.setSliders(SLIDER_I_OUT);
                        flag2 = true;
                    }
                    if(gamepad1.dpad_right){
                        intake.setSliders(SLIDER_I_LASTQUARTER);
                        flag2 = true;
                    }
                    if(gamepad1.dpad_left){
                        intake.setSliders(SLIDER_I_HALF);
                        flag2 = true;
                    }
                    if(gamepad1.dpad_down){
                        intake.setSliders(SLIDER_I_QUARTER);
                        flag2 = true;
                    }

                    if(gamepad1.dpad_right && flag && offsetMuneca != 0.15 && cd.milliseconds() > 200){
                        offsetMuneca = offsetMuneca + 0.025;
                        intake.setMuneca(offsetMuneca);
                        cd.reset();
                    }
                    if(gamepad1.dpad_left && flag && offsetMuneca != 0 && cd.milliseconds() > 200){
                        offsetMuneca = offsetMuneca - 0.025;
                        intake.setMuneca(offsetMuneca);
                        cd.reset();
                    }

                    if((gamepad1.a && flag2)){
                        robotState = RobotState.CLOSE_INTAKE;
                        timer.reset();
                    }
                    if((gamepad1.b && flag2)){
                        robotState = RobotState.IDLE;
                        timer.reset();
                    }

                    break;

                case CLOSE_INTAKE:
                    state = "CLOSE_INTAKE";
                    if(timer.milliseconds() < 200){
                        intake.setMuneca(MUNECA_I_HOR);
                        open = true;
                    }
                    if(timer.milliseconds() > 200 && timer.milliseconds() < 400){
                        intake.setRotation(BRAZO_IN_INTAKING);
                    }

                    if(gamepad1.a && cd.milliseconds() > 200){
                        if(open){
                            intake.setGarra(GARRA_CERRADA_I);
                            open = false;
                            cd.reset();
                        } else {
                            intake.setGarra(GARRA_ABIERTA_O);
                            open = true;
                            cd.reset();
                        }
                    }

                    if(timer.milliseconds() > 400 && !open){
                        if(gamepad1.x){
                            robotState = RobotState.PRE_TRANSFER;
                            timer.reset();
                        }
                    }
                    if(timer.milliseconds() > 400){
                        if(gamepad1.b){
                            robotState = RobotState.INTAKING;
                        }
                    }

                    break;

                case PRE_TRANSFER:
                    state = "PRE_TRANSFER";
                    if(timer.milliseconds() < 200){
                        intake.setSliders(SLIDER_I_IN);
                        intake.setRotation(BRAZO_IN_PREINTAKE);
                    }

                    if(timer.milliseconds() > 200){
                        if(gamepad1.a){
                            robotState = RobotState.TRANSFER;
                            timer.reset();
                        }
                        if(gamepad1.b){
                            robotState = RobotState.DROP;
                            timer.reset();
                        }
                    }

                    break;

                case DROP:
                    state = "DROP";
                    if(timer.milliseconds() < 200){
                        intake.setRotation(BRAZO_IN_INTAKING);
                    }
                    if(timer.milliseconds() > 200){
                        intake.setGarra(GARRA_ABIERTA_I);
                    }

                    if(timer.seconds() > 4){
                        robotState = RobotState.PRE_TRANSFER;
                        timer.reset();
                    }
                    break;

                case TRANSFER:
                    state = "TRANSFER";
                    if(timer.milliseconds() < 500){
                        // gira intake
                        intake.setRotation(BRAZO_IN_INTAKING);
                        intake.setInnerRotation(ROT_IN_TRANSFER);
                    }
                    if(timer.milliseconds() > 500 && timer.milliseconds() < 1000){
                        outake.setRotation(BRAZO_OUT_TRANSFER);
                        outake.setInnerRotation(ROT_OUT_TRANSFER);
                        outake.setGarra(GARRA_ABIERTA_O);
                    }
                    if(timer.milliseconds() > 1000 && timer.milliseconds() < 1300){
                        outake.setGarra(GARRA_CERRADA_O);
                    }
                    if(timer.milliseconds() > 1300 && timer.milliseconds() < 1600){
                        intake.setGarraAction(GARRA_ABIERTA_I);
                    }
                    if(timer.milliseconds() > 1600){
                        outake.setRotation(BRAZO_OUT_SCORING);

                        // servos de intake en posicion inicial
                        intake.setSliders(SLIDER_I_IN);
                        intake.setRotation(BRAZO_IN_IDLE);
                        intake.setGarra(GARRA_ABIERTA_I);
                        intake.setInnerRotation(ROT_IN_DOWN);
                        intake.setMuneca(MUNECA_I_HOR);

                        if(gamepad2.a){
                            robotState = RobotState.OUTAKING;
                            timer.reset();
                        }
                        if(gamepad1.b || gamepad2.b){
                            robotState =  RobotState.IDLE;
                            timer.reset();
                        }
                    }
                    break;

                case OUTAKING:
                    state = "OUTAKING";
                    if(timer.milliseconds() < 200){
                        outake.setInnerRotation(ROT_OUT_START);
                        gamepad2.rumble(200);
                    }

                    if(timer.milliseconds() > 200){
                        targetOutake = CLIMBER_ABAJO;

                        if(gamepad2.right_bumper){
                            robotState = RobotState.RAISE1;
                            timer.reset();
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
                    state = "RAISE1";
                    // levantar el climber a la altura de la primera canasta
                    targetOutake = CLIMBER_F_BASKET;

                    if(timer.milliseconds() > 200){
                        if(gamepad2.left_bumper){
                            robotState = RobotState.OUTAKING;
                        }
                        if(gamepad2.right_bumper){
                            robotState = RobotState.RAISE2;
                        }
                        if(gamepad2.a){
                            robotState = RobotState.DROPPING;
                        }
                    }

                    break;

                case RAISE2:
                    state = "RAISE2";
                    // levantar el climber a la altura de la segunda canasta
                    targetOutake = CLIMBER_S_BASKET-500;

                    if(gamepad2.left_bumper){
                        robotState = RobotState.RAISE1;
                    }
                    if(gamepad2.a){
                        robotState = RobotState.DROPPING;
                    }
                    break;

                case DROPPING:
                    state = "DROPPING";
                    outake.setGarra(GARRA_ABIERTA_O);

                    if(gamepad2.b){
                        robotState = RobotState.OUTAKING;
                    }
                    break;

                case SPECIMING:
                    state = "SPECIMEN";
                    if(timer.milliseconds() < 200){
                        targetOutake = CLIMBER_SLIGHTLY;
                    }
                    if(timer.milliseconds() > 200 && timer.milliseconds() < 300){
                        outake.setRotation(BRAZO_OUT_SPECIMEN);
                        outake.setInnerRotation(ROT_OUT_OTTAKE);
                        outake.setGarra(GARRA_ABIERTA_O);
                    }

                    if(timer.milliseconds() > 300){
                        if(gamepad1.x){
                            robotState = RobotState.CLOSE_S;
                            timer.reset();
                        }
                        if(gamepad1.b){
                            robotState = RobotState.IDLE;
                            timer.reset();
                        }
                    }
                    break;

                case CLOSE_S:
                    state = "CLOSE_S";
                    if(timer.milliseconds() < 200){
                        openS = true;
                    }
                    targetOutake = CLIMBER_SLIGHTLY;

                    if(gamepad1.a && cd.milliseconds() > 200){
                        if(openS){
                            outake.setGarra(GARRA_CERRADA_O);
                            openS = false;
                            cd.reset();
                        } else {
                            outake.setGarra(GARRA_ABIERTA_O);
                            openS = true;
                            cd.reset();
                        }
                    }

                    if(timer.milliseconds() > 200){
                        if(gamepad1.x){
                            robotState = RobotState.PRE_OUTAKING_S;
                            timer.reset();

                        }
                        if(gamepad1.b){
                            robotState = RobotState.SPECIMING;
                        }
                    }

                    break;

                case PRE_OUTAKING_S:
                    state = "PRE_OUTAKING_S";
                    // levanta un poquito el climber
                    targetOutake = CLIMBER_SLIGHTLY-400;
                    if(timer.milliseconds() > 200){
                        if(gamepad1.x){
                            robotState = RobotState.OUTAKING_S;
                            timer.reset();
                        }
                        if(gamepad1.b){
                            robotState = RobotState.CLOSE_S;
                        }
                    }

                    break;


                case OUTAKING_S:

                    state = "OUTAKING_S";
                    if(timer.milliseconds() < 200){
                        outake.setRotation(BRAZO_OUT_SCORING);
                        outake.setInnerRotation(ROT_OUT_OTLEAVE);
                        // MUNECA 180°
                    }

                    if(timer.milliseconds() > 200){
                        targetOutake = CLIMBER_ABAJO;
                        if(gamepad2.right_bumper){
                            robotState = RobotState.DROPPING_S;
                            timer.reset();
                        }
                    }

                    break;

                case DROPPING_S:
                    state = "DROPPING_S";
                    outake.setGarra(GARRA_ABIERTA_O);

                        if(gamepad2.a){
                            robotState = RobotState.RESET_OUTAKE;
                            timer.reset();
                    }

                    break;

                case RESET_OUTAKE:
                    state = "RESET_OUTAKE";
                    if(timer.seconds() < 1){
                        // servos de outake en posicion inicial
                        outake.setRotation(ROT_OUT_START);
                        outake.setMuneca(MUNECA_O_HOR);
                        outake.setGarra(GARRA_ABIERTA_O);
                        outake.setInnerRotation(BRAZO_OUT_IDLE);
                    }

                    // brazo de outake default
                    if(timer.seconds() > 1){
                        if(gamepad2.a){
                            robotState = RobotState.IDLE;
                            timer.reset();
                        }
                        if(gamepad2.x){
                            robotState = RobotState.SPECIMING;
                            timer.reset();
                        }
                    }
                    break;
            }

            if(gamepad1.left_trigger > 0.8){
                velocity = 0.15;
                velocity_lat = 0.25;
                velocity_rot = 0.15;
            }
            if(gamepad1.left_bumper){
                velocity = 0.3;
                velocity_lat = 0.5;
                velocity_rot = 0.4;
            }
            if(gamepad1.right_bumper){
                velocity = 0.5;
                velocity_lat = 0.7;
                velocity_rot = 0.6;
            }
            if(gamepad1.right_trigger > 0.8){
                velocity = 0.8;
                velocity_lat = 1;
                velocity_rot = 0.9;
            }

            if(gamepad2.y){
                if(!nuke && cd.milliseconds() > 200){
                    nuke = true;
                    cd.reset();
                } else if (nuke && cd.milliseconds() > 200) {
                    nuke = false;
                    cd.reset();
                }
            }
            if(gamepad2.dpad_up && nuke){
                if(cd.milliseconds() > 200 && targetOutake != 3000){
                    targetOutake = targetOutake - 1000;
                    cd.reset();
                }
            }
            if(gamepad2.dpad_down && nuke){
                targetOutake = -1800;
            }




            drive.updatePoseEstimate();
            climber.updatePID(targetOutake);

            telemetry.addData("current state: ", state);
            telemetry.addData("Climber unlocked: ", nuke);
            telemetry.update();

        }
    }
}