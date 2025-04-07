package org.firstinspires.ftc.teamcode.limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "Lime Custom Python")
public class Data extends LinearOpMode {

    private Limelight3A limelight;
    double lastAngle = 0.0;
    double servoPosition = 0.5;
    double angleMargin = 0.8;
    double deadZone = 2.0;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        Servo servo = hardwareMap.get(Servo.class, "servo1");

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(9);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();

            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();

            double ty = result.getTy();

            if (Math.abs(ty - lastAngle) > angleMargin + deadZone) {
                double adjustedAngle = ty * 2;

                servoPosition = (adjustedAngle + 180) / 360;

                servoPosition = Math.min(1.0, Math.max(0.0, servoPosition));
                servo.setPosition(servoPosition);
                lastAngle = ty;
            }else{
                servo.setPosition(servoPosition);
            }

            telemetry.addData("txnc", result.getTxNC());
            telemetry.addData("tx", result.getTx());
            telemetry.addData("tync", result.getTyNC());
            telemetry.addData("Angle (ty)", result.getTy());
            telemetry.addData("SERVO ANGLE: ", servoPosition);

            telemetry.update();
        }
    }
}