package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class PIDAction implements Action {
    Climber climber;

    public PIDAction(Climber climber){
        this.climber = climber;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        climber.updatePIDAction();
        return true;
    }
}
