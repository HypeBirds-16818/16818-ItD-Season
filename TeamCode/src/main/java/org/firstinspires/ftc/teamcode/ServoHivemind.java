package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class ServoHivemind implements Action {
    public static double ROT_OUT_CERO = 0.97;
    public static double ROT_OUT_CIENTOCHENTA = 0;
    public static int CLIMBER_ABAJO = -50;
    public static int CLIMBER_SLIGHTLY = -300;
    public static int CLIMBER_F_BASKET = -1000;
    public static int CLIMBER_S_BASKET = -3100;
    public static int CLIMBER_RUNG = -1500;
    public static double BRAZO_OUT_ABAJO = 0.05;
    public static double BRAZO_OUT_MEDIO = 0.3;
    public static double BRAZO_OUT_ARRIBA = 0.7;
    public static double BRAZO_OUT_ATRAS = 1;
    public static double GARRA_ABIERTA_I = 0.6;
    public static double GARRA_CERRADA_I = 0.99;

    public static double GARRA_ABIERTA_O = 0;
    public static double GARRA_CERRADA_O = 0.19;
    public static int SLIDER_I_IN = 0;
    public static int SLIDER_I_OUT = 500;
    public static double ROT_IN_CERO = 0.97;
    public static double ROT_IN_NOVENTA = 0.645;
    public static double ROT_IN_CIENTOCHENTA = 1;
    public static double BRAZO_IN_CERO = 1;
    public static double BRAZO_IN_NOVENTA = 0.6;
    public static double BRAZO_IN_CIENTOCHENTA = 0.13;
    public static double MUNECA_VER = 0.65;
    public static double MUNECA_HOR = 0.97;
    public static double ir = ROT_IN_CERO;
    public static double im = MUNECA_VER;
    public static double ib = BRAZO_IN_CERO;
    public static double or = ROT_OUT_CERO;
    public static double ob = BRAZO_OUT_MEDIO;
    public static double ig = GARRA_ABIERTA_I;
    public static double og = GARRA_ABIERTA_O;

    Intake intake;
    Outtake outake;

    public ServoHivemind(Intake intake, Outtake outake){
        this.intake = intake;
        this.outake = outake;
    }


    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {



        return true;
    }
}
