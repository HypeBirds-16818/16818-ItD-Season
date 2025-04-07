package org.firstinspires.ftc.teamcode.limelight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

@Config
@TeleOp(name = "TEST LIME", group = "Sensor")
public class Limelight extends LinearOpMode {
    private Limelight3A limelight;
    private Servo servo;
    public static double alpha = 0.15;  // Incrementado para hacer la respuesta más rápida
    public static double DEAD_ZONE = 0.5;
    public static double MAX_DELTA = 2.0;
    public static double MIN_SERVO_POS = 0.0;
    public static double MAX_SERVO_POS = 1.0;
    public static double MAX_ANGLE = 90.0;
    private double filteredAngle = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(11);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        servo = hardwareMap.get(Servo.class, "servo1");
        servo.setDirection(Servo.Direction.REVERSE);

        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Listo. Esperando Start...");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();

                if (!colorResults.isEmpty()) {
                    double rawAngle = colorResults.get(0).getTargetXDegrees();

                    if (Math.abs(rawAngle - filteredAngle) > DEAD_ZONE) {
                        double delta = rawAngle - filteredAngle;

                        // Limitar el cambio de ángulo
                        delta = Math.max(-MAX_DELTA, Math.min(MAX_DELTA, delta));

                        // Filtrado de ángulo suave
                        filteredAngle += alpha * delta;
                    }

                    double servoPos = mapAngleToServo(filteredAngle);
                    servo.setPosition(servoPos);

                    telemetry.addData("Raw Angle", rawAngle);
                    telemetry.addData("Filtered Angle", filteredAngle);
                    telemetry.addData("Servo Position", servoPos);
                } else {
                    telemetry.addLine("No se detecta objeto.");
                }
            } else {
                telemetry.addLine("No hay datos de Limelight.");
            }
            telemetry.update();
        }
        limelight.stop();
    }

    private double mapAngleToServo(double angle) {
        // Limitar el ángulo para que esté dentro del rango aceptable
        angle = Math.max(-MAX_ANGLE, Math.min(MAX_ANGLE, angle));
        double normalized = (angle + MAX_ANGLE) / (2 * MAX_ANGLE);
        return MIN_SERVO_POS + normalized * (MAX_SERVO_POS - MIN_SERVO_POS);
    }
}