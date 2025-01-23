package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Actions.DualServoAction;
import org.firstinspires.ftc.teamcode.Actions.ServoAction;

public class Intake {
    private Servo sliderLeft;
    private Servo sliderRight;
    private Servo rotationLeft;
    private Servo rotationRight;
    private Servo diffLeft;
    private Servo diffRight;
    private Servo garraIntake;


    public Intake(HardwareMap hardwareMap){


    }

    public void init(HardwareMap hardwareMap){
        sliderLeft = hardwareMap.get(Servo.class, "sliderLeft");
        sliderRight = hardwareMap.get(Servo.class, "sliderRight");
        rotationLeft = hardwareMap.get(Servo.class, "rotationLeft");
        rotationRight = hardwareMap.get(Servo.class, "rotationRight");
        diffLeft = hardwareMap.get(Servo.class, "diffLeft");
        diffRight = hardwareMap.get(Servo.class, "diffRight");
        garraIntake = hardwareMap.get(Servo.class, "garraIntake");

//        sliderLeft.setPosition(0);
//        sliderRight.setPosition(0);
//        rotationLeft.setPosition(0);
//        rotationRight.setPosition(0);
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
        garraIntake.setPosition(position);
    }

    public void setMuneca(double position) {
        diffLeft.setPosition(position);
        diffRight.setPosition(1 - position);
    }

    public void setSliders(double position){
        sliderRight.setPosition(position);
        sliderLeft.setPosition(position);
    }

    public Action setRotationAction(double position){
        return new DualServoAction(rotationLeft, rotationRight, position, position);
    }

    public Action setInnerRotationAction(double position){
        return new DualServoAction(diffLeft, rotationRight, position, position);
    }

    public Action setGarraAction(double position){
        return new ServoAction(garraIntake, position);
    }

    public Action setMunecaAction(double position){
        return new DualServoAction(diffLeft, rotationRight, position, 1-position);
    }

    public Action setSlidersAction(double position){
        return new DualServoAction(sliderLeft, sliderRight, position, position);
    }


}
