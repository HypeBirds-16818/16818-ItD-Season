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

    }

    public void init(){
        controller = new PIDController(p,i,d);
        rotationIntake.setPosition(1);
        garraIntake.setPosition(1);
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setMotorMode(DcMotor.RunMode runMode){
        poleasIntake.setMode(runMode);
    }

    public void updatePID(){
        controller.setPID(p,i,d);
        int intakePos = poleasIntake.getCurrentPosition();

        double pid = controller.calculate(intakePos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_per_degree)) * f;
        double power = pid + ff;

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

    public void openGarra(){
        garraIntake.setPosition(1);
    }

    public void closeGarra(){
        garraIntake.setPosition(0);
    }

}
