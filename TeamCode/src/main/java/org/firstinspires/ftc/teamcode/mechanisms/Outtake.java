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

        diffLeft.setPosition(diffLeftPos - offset);
        diffRight.setPosition(diffRightPos + offset);
    }

    public Action setRotationAction(double position){
        diffLeftPos = position;
        diffRightPos = position;
        return new DualServoAction(diffLeft, diffRight, diffLeftPos - offset, diffRightPos + offset);
    }

    public Action setInnerRotationAction(double position){
        return new DualServoAction(diffLeft, diffRight, position, position);
    }

    public Action setGarraAction(double position){
        return new ServoAction(garraOuttake, position);
    }

    public Action setMunecaAction(double offset){
        this.offset = offset;
        return new DualServoAction(diffLeft, diffRight, diffLeftPos - offset, diffRightPos + offset);
    }

}
