package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@Config
@TeleOp
public class direction_test extends OpMode {

    private DcMotorEx motor1;
    private DcMotorEx motor2;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");

        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

        int motor1pos = motor1.getCurrentPosition();
        int motor2pos = motor2.getCurrentPosition();

        int slidepos = (motor1pos + motor2pos)/2;

        telemetry.addData("pos 1", motor1pos);
        telemetry.addData("pos 2", motor2pos);
        telemetry.addData("pos avg", slidepos);
        telemetry.update();

    }
}
