package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class PID_Test_intake extends OpMode {
    private PIDController controller;

    public static double p = 0.004, i = 0, d = 0.0001;
    public static double f = 0.1;

    public static int target = 0;
    private final double ticks_per_degree = 537.7/180;

    private DcMotorEx motor1;
    //private DcMotorEx piston;

    @Override
    public void init() {
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor1 = hardwareMap.get(DcMotorEx.class, "sliderMotor");
        //piston = hardwareMap.get(DcMotorEx.class, "piston");
        motor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        controller.setPID(p,i,d);
        int motor1pos = motor1.getCurrentPosition();

        int slidepos = motor1pos;

        double pid = controller.calculate(slidepos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_per_degree)) * f;
        double power = pid;

        motor1.setPower(power);



        telemetry.addData("pos 1", motor1pos);
        telemetry.addData("pos avg", slidepos);
        telemetry.addData("target ", target);
        telemetry.update();



    }
}
