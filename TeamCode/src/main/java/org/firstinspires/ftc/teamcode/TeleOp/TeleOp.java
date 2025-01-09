package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Climber;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Outtake;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpToluca")
public class TeleOp extends LinearOpMode {
    public static int targetIntake = 0;
    public static int targetOutake = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Intake intake = new Intake(hardwareMap);
        Outtake outake = new Outtake(hardwareMap);
        Climber climber = new Climber(hardwareMap);


        waitForStart();
        intake.init();
        outake.init();
        climber.init();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

        }
    }
}
