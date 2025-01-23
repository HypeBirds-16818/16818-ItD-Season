package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Actions.DualServoAction;
import org.firstinspires.ftc.teamcode.Actions.ServoAction;

public class Outtake {
    private Servo rotationLeft;
    private Servo rotationRight;
    private Servo garraOuttake;
    private Servo diffLeft;
    private Servo diffRight;

    public Outtake(HardwareMap hardwareMap){

    }

    public void init(HardwareMap hardwareMap){
        rotationLeft = hardwareMap.get(Servo.class, "rotationLeft");
        rotationRight = hardwareMap.get(Servo.class, "rotationRight");
        garraOuttake = hardwareMap.get(Servo.class, "garraOuttake");
        diffLeft = hardwareMap.get(Servo.class, "diffLeft");
        diffRight = hardwareMap.get(Servo.class, "diffRight");

//        rotationLeft.setPosition(0);
//        rotationRight.setPosition(0);
//        garraOuttake.setPosition(0);
//        diffLeft.setPosition(0);
//        diffRight.setPosition(0);

        // Reverse servos that are necessary

    }

    public void setRotation(double position){
        rotationLeft.setPosition(position);
        rotationRight.setPosition(position);
    }

    public void setInnerRotation(double position){
        diffLeft.setPosition(position);
        diffRight.setPosition(position);
    }

    public void setGarra(double position){
        garraOuttake.setPosition(position);
    }

    public void setMuneca(double position) {
        diffLeft.setPosition(position);
        diffRight.setPosition(1 - position);
    }

    public Action setRotationAction(double position){
        return new DualServoAction(rotationLeft, rotationRight, position, position);
    }

    public Action setInnerRotationAction(double position){
        return new DualServoAction(diffLeft, diffRight, position, position);
    }

    public Action setGarraAction(double position){
        return new ServoAction(garraOuttake, position);
    }

    public Action setMunecaAction(double position){
        return new DualServoAction(diffLeft, diffRight, position, 1 - position);
    }

}
