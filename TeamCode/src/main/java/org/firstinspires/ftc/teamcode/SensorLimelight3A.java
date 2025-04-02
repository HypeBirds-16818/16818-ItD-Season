package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
import java.util.ArrayList;

@TeleOp(name = "Sensor: Limelight3A", group = "Sensor")

public class SensorLimelight3A extends LinearOpMode {

    private Limelight3A limelight;

    // Configurable parameters for sample and camera setup
    private final double idealAspectRatio = 1.5 / 3.5; // Expected width:height ratio for a properly aligned sample
    private final double angleWeight = 15.0; // Weight for scoring based on rotation
    private final double minDistance = 18.0; // Minimum allowable distance (adjust as needed)
    private final double maxDistance = 32.0;
    private final double limelightHeight = 12.25; //how high up the limelight is in inches;
    private final double limelightAngle = 58.5125; //angle of the limelight, camera facing straight down would be 0
    //and camera facing straight forwards would be 90

    private double limelightRotationOffset = 90.0; // Rotation of the camera in degrees

    private double yDistance;
    private double xDistance;

    private final double forwardOffset = 4;
    private final double lateralOffset = -4.75;


    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.setMsTransmissionInterval(11);


        double sampleDir=90;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8); //Red sample pipeline
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                // Access general information
                Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);
                telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

                if (result.isValid()) {
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("txnc", result.getTxNC());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("tync", result.getTyNC());


                    boolean checked=false;
                    // Access color results
                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                    for (LLResultTypes.ColorResult cr : colorResults) {
                        telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                        telemetry.addData("Corner number: ", cr.getTargetCorners().size());

                        // Getting the target corners (list of 4 corners of the bounding box)
                        List<List<Double>> targetCorners = cr.getTargetCorners();
                        if(checked==false){
                            // Check if there are 4 corners (to form a bounding box)
                            if (targetCorners.size() >= 4) {
                                // Calculate aspect ratio and rotation
                                double aspectRatio = calculateDistance(targetCorners.get(0), targetCorners.get(1)) / calculateDistance(targetCorners.get(1), targetCorners.get(2));
                                telemetry.addData("Aspect ratio angle", normalize(aspectRatio,1.5/3.5,3.5/1.5));
                            } else {
                                telemetry.addData("Bounding Box", "Not enough corners to calculate.");
                            }
                        }checked=true;
                    }
                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }

            telemetry.update();
        }
        limelight.stop();
    }

    //region Utility methods
    // Method to calculate the distance between two points
    public static double pointDistance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }

    private double calculateDistance(List<Double> point1, List<Double> point2) {
        double dx = point1.get(0) - point2.get(0);
        double dy = point1.get(1) - point2.get(1);
        return Math.sqrt(dx * dx + dy * dy);
    }

    // Method to calculate the direction (angle) between two points
    public static double pointDirection(double x1, double y1, double x2, double y2) {
        return Math.toDegrees(Math.atan2(y2 - y1, x2 - x1));
    }

    public static double normalize(double value, double min, double max) {
        return (value - min) / (max - min);
    }
 //endregoin
}