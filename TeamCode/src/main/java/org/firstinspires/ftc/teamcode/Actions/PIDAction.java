package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.mechanisms.Climber;

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
