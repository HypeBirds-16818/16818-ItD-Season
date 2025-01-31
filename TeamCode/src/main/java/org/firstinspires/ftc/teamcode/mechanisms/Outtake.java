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

    private double diffLeftPos;
    private double diffRightPos;

    private double offset = 0;

    public Outtake(HardwareMap hardwareMap){

    }

    public void init(HardwareMap hardwareMap){
        rotationLeft = hardwareMap.get(Servo.class, "rotationLeftO");
        rotationRight = hardwareMap.get(Servo.class, "rotationRightO");
        garraOuttake = hardwareMap.get(Servo.class, "garraOuttake");
        diffLeft = hardwareMap.get(Servo.class, "diffLeftO");
        diffRight = hardwareMap.get(Servo.class, "diffRightO");

//        rotationLeft.setPosition(0);
//        rotationRight.setPosition(0);
//        garraOuttake.setPosition(0);
//        diffLeft.setPosition(0);
//        diffRight.setPosition(0);

        // Reverse servos that are necessary
        rotationLeft.setDirection(Servo.Direction.REVERSE);
        diffRight.setDirection(Servo.Direction.REVERSE);


    }

    public void setRotationLeft(double position){
        rotationLeft.setPosition(position);
    }

    public void setRotationRight(double position){
        rotationRight.setPosition(position);
    }

    public void setDiffLeft(double position){
        diffLeft.setPosition(position);
    }
    public void setDiffRight(double position){
        diffRight.setPosition(position);
    }

    public void setRotation(double position){
        rotationLeft.setPosition(position);
        rotationRight.setPosition(position);
    }

    public void setInnerRotation(double position){
        diffLeftPos = position;
        diffRightPos = position;

        diffLeft.setPosition(diffLeftPos - offset);
        diffRight.setPosition(diffRightPos + offset);
    }

    public void setGarra(double position){
        garraOuttake.setPosition(position);
    }

    public void setMuneca(double offset) {
        this.offset = offset;

        diffLeft.setPosition(diffLeftPos - this.offset);
        diffRight.setPosition(diffRightPos + this.offset);
    }

    public Action setRotationAction(double position){
        return new DualServoAction(rotationLeft, rotationRight, position, position);
    }

    public Action setInnerRotationAction(double position){
        diffLeftPos = position;
        diffRightPos = position;
        return new DualServoAction(diffLeft, diffRight, diffLeftPos - offset, diffRightPos + offset);
    }

    public Action setGarraAction(double position){
        return new ServoAction(garraOuttake, position);
    }

    public Action setMunecaAction(double offset){
        this.offset = offset;
        return new DualServoAction(diffLeft, diffRight, diffLeftPos - offset, diffRightPos + offset);
    }

}
