package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Actions.DualServoAction;
import org.firstinspires.ftc.teamcode.Actions.ServoAction;

public class Intake {

    private Servo rotationLeft;
    private Servo rotationRight;
    private Servo diffLeft;
    private Servo diffRight;
    private Servo garraIntake;
    private DcMotorEx sliderMotor;
    private DigitalChannel magneticSwitch;

    private PIDController controller;

    private static double p = 0.004, i = 0, d = 0.0001;
    public static double f = 0.1;

    private static int target = 0;

    public int slidepos;
    private final double ticks_per_degree = 537.7;

    private double diffLeftPos;
    private double diffRightPos;

    private double offset = 0;


    public Intake(HardwareMap hardwareMap){

    }

    public void init(HardwareMap hardwareMap){
        rotationLeft = hardwareMap.get(Servo.class, "rotationLeftI");
        rotationRight = hardwareMap.get(Servo.class, "rotationRightI");
        diffLeft = hardwareMap.get(Servo.class, "diffLeftI");
        diffRight = hardwareMap.get(Servo.class, "diffRightI");
        garraIntake = hardwareMap.get(Servo.class, "garraIntake");
        sliderMotor = hardwareMap.get(DcMotorEx.class, "sliderMotor");
        magneticSwitch = hardwareMap.get(DigitalChannel.class, "magnetic");

        magneticSwitch.setMode(DigitalChannel.Mode.INPUT);
        controller = new PIDController(p,i,d);
        sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        sliderLeft.setPosition(0);
//        sliderRight.setPosition(0);
//        rotationLeft.setPosition(0);
//        rotationRight.setPosition(0);
//        diffLeft.setPosition(0);
//        diffRight.setPosition(0);


        // Reverse servos that are necessary
        diffRight.setDirection(Servo.Direction.REVERSE);
        rotationRight.setDirection(Servo.Direction.REVERSE);

    }

    public boolean checkMagnetic() {
        return !magneticSwitch.getState();
    }

    public void setMode(DcMotor.RunMode runMode){
        sliderMotor.setMode(runMode);
    }

    public void setRotation(double position){
        rotationLeft.setPosition(position);
        rotationRight.setPosition(position);
    }

    public void setPower(double power){
        sliderMotor.setPower(power);
    }

    public void setInnerRotation(double position){
        diffLeftPos = position;
        diffRightPos = position;

        diffLeft.setPosition(diffLeftPos - offset);
        diffRight.setPosition(diffRightPos + offset);
}

    public void setGarra(double position){
        garraIntake.setPosition(position);
    }

    public void setMuneca(double offset) {
        this.offset = offset;

        diffLeft.setPosition(diffLeftPos - offset);
        diffRight.setPosition(diffRightPos + offset);
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
        return new ServoAction(garraIntake, position);
    }

    public Action setMunecaAction(double offset){
        this.offset = offset;

        return new DualServoAction(diffLeft, diffRight, diffLeftPos - offset, diffRightPos + offset);
    }


    public int getIntakePosition(){
        return sliderMotor.getCurrentPosition();
    }

    public void updatePID(int tTarget){
        controller.setPID(p,i,d);
        int slidepos = sliderMotor.getCurrentPosition();

        double pid = controller.calculate(slidepos, tTarget);
        double power = pid;

        sliderMotor.setPower(power);
    }

    public Action updatePIDAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                controller.setPID(p,i,d);
                int motor1pos = sliderMotor.getCurrentPosition();

                slidepos = motor1pos;

                double pid = controller.calculate(slidepos, target);
                double power = pid;

                sliderMotor.setPower(power);
                return true;
            }
        };
    }

    public Action setModeAction(DcMotor.RunMode runMode){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                sliderMotor.setMode(runMode);
                return false;
            }
        };

    }

    public void setTarget(int newTarget){
        target = newTarget;
    }

    public Action setTargetAction(int newTarget){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                target = newTarget;
                return false;
            }
        };
    }


}
