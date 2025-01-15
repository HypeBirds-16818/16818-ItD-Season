package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

public class Intake {
    public Servo rotationIntake;
    public Servo garraIntake;
    public Servo brazoIntake;
    public Servo munecaIntake;

    public DcMotorEx poleasIntake;
    public PIDController controller;

    public static double p = 0.004, i = 0, d = 0.0001;
    public static double f = 0.1;
    public int target = 0;
    private final double ticks_per_degree = 537.7/180;

    public Intake(HardwareMap hardwareMap){
        rotationIntake = hardwareMap.get(Servo.class, "rotationIntake");
        brazoIntake = hardwareMap.get(Servo.class, "brazoIntake");
        garraIntake = hardwareMap.get(Servo.class, "garraIntake");
        poleasIntake = hardwareMap.get(DcMotorEx.class, "intake");
        munecaIntake = hardwareMap.get(Servo.class, "munecaIntake");

    }

    public void init(){
        controller = new PIDController(p,i,d);
        rotationIntake.setPosition(1);
        garraIntake.setPosition(1);
        munecaIntake.setPosition(1);
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setMotorMode(DcMotor.RunMode runMode){
        poleasIntake.setMode(runMode);
    }

    public void updatePID(int Ttarget){
        controller.setPID(p,i,d);
        int intakePos = poleasIntake.getCurrentPosition();

        double pid = controller.calculate(intakePos, Ttarget);
        double power = pid;

        poleasIntake.setPower(power);
    }

    public void setTarget(int newTarget){
        target = newTarget;
    }

    public void setRotation(double position){
        rotationIntake.setPosition(position);
    }

    public void setBrazo(double position){
        brazoIntake.setPosition(position);
    }

    public void setGarra(double position){
        garraIntake.setPosition(position);
    }



    public void setMuneca(double position){ munecaIntake.setPosition(position);}

}
