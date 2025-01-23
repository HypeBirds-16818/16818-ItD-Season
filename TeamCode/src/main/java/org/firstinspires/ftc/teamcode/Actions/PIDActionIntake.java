package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;

public class PIDActionIntake implements Action {
    Intake intake;

    public PIDActionIntake(Intake intake){
        this.intake = intake;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        intake.updatePIDAction();
        return true;
    }
}
