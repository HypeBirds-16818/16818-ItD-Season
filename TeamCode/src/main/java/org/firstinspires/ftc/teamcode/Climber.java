package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Climber {
    private PIDController controller;

    public static double p = 0.004, i = 0, d = 0.0001;
    public static double f = 0.1;

    public static int target = 0;

    public double pistonpw = 0;
    public int slidepos;
    private final double ticks_per_degree = 537.7/180;

    private DcMotorEx motor1;
    private DcMotorEx motor2;
    private DcMotorEx piston;

    public Climber(HardwareMap hardwareMap){

    }

    public void init(HardwareMap hardwareMap){
        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        piston = hardwareMap.get(DcMotorEx.class, "piston");
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        controller = new PIDController(p,i,d);
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setMotorMode(DcMotor.RunMode runMode){
        motor1.setMode(runMode);
        motor2.setMode(runMode);
    }

    public void updatePID(int Ttarget){
        controller.setPID(p,i,d);
        int motor1pos = motor1.getCurrentPosition();
        int motor2pos = motor2.getCurrentPosition();

        slidepos = (motor1pos + motor2pos)/2;

        double pid = controller.calculate(slidepos, Ttarget);
        double ff = Math.cos(Math.toRadians(Ttarget / ticks_per_degree)) * f;
        double power = pid + ff;

        motor1.setPower(power);
        motor2.setPower(power);
    }

    public Action updatePIDAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                controller.setPID(p,i,d);
                int motor1pos = motor1.getCurrentPosition();
                int motor2pos = motor2.getCurrentPosition();

                slidepos = (motor1pos + motor2pos)/2;

                double pid = controller.calculate(slidepos, target);
                double ff = Math.cos(Math.toRadians(target / ticks_per_degree)) * f;
                double power = pid + ff;

                motor1.setPower(power);
                motor2.setPower(power);
                return true;
            }
        };
    }

    public Action setTarget(int newTarget){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                target = newTarget;
                return false;
            }
        };
    }

    public void setPistonSpeed(double speed){
        piston.setPower(speed);
    }

    public int getSlidePos(){
        return slidepos;
    }
}
