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
public class PID_Test extends OpMode {
    private PIDController controller;

    public static double p = 0.004, i = 0, d = 0.0001;
    public static int target = 0;
    private final double ticks_per_degree = 537.7/180;

    private DcMotorEx motor2;

    @Override
    public void init() {
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor2 = hardwareMap.get(DcMotorEx.class, "intake");

        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void loop() {
        controller.setPID(p,i,d);
        int slidepos = motor2.getCurrentPosition();

        double pid = controller.calculate(slidepos, target);
        double power = pid;

        motor2.setPower(power);

        telemetry.addData("pos avg", slidepos);
        telemetry.addData("target ", target);
        telemetry.update();



    }
}
