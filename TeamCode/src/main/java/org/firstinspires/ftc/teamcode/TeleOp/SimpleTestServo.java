package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class SimpleTestServo extends LinearOpMode {
    Servo garraOutake;

    @Override
    public void runOpMode() throws InterruptedException {
        garraOutake = hardwareMap.get(Servo.class, "brazoOuttake");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            garraOutake.setPosition(0.5);
        }
    }
}
