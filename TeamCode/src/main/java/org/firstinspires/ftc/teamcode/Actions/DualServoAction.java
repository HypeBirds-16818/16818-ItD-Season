package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

public class DualServoAction implements Action {
    Servo servo1;
    Servo servo2;
    double position1;
    double position2;

    public DualServoAction(Servo servo1, Servo servo2, double position1, double position2){
        this.servo1 = servo1;
        this.servo2 = servo2;
        this.position1 = position1;
        this.position2 = position2;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        servo1.setPosition(position1);
        servo2.setPosition(position2);
        return false;
    }
}
