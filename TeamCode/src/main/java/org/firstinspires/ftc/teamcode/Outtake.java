package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {
    public Servo rotationOuttake;
    public Servo garraOuttake;
    public Servo brazoOuttake;

    public Outtake(HardwareMap hardwareMap){
        rotationOuttake = hardwareMap.get(Servo.class, "rotationOuttake");
        garraOuttake = hardwareMap.get(Servo.class, "garraOuttake");
        brazoOuttake = hardwareMap.get(Servo.class, "brazoOuttake");
    }

    public void init(){
        rotationOuttake.setPosition(0);
        garraOuttake.setPosition(0);
        brazoOuttake.setPosition(0);
    }

    public void setRotation(double position){
        rotationOuttake.setPosition(position);
    }

    public void setBrazo(double position){
        brazoOuttake.setPosition(position);
    }

    public void setGarra(double position){
        garraOuttake.setPosition(position);
    }

}
