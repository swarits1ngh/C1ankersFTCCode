package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;


import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Core;
import org.opencv.imgproc.Imgproc;


import java.util.ArrayList;
import java.util.List;


@Autonomous(name = "StarterBot Green/Purple Follow Auto", group = "StarterBot")
public class StarterBotAutonomous extends LinearOpMode {


    private DcMotor leftDrive, rightDrive;
    private OpenCvCamera webcam;
    private BallDetectionPipeline pipeline;


    @Override
    public void runOpMode() throws InterruptedException {


        // ------------------------------
        // Motor setup
        // ------------------------------
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");


        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);


        // ------------------------------
        // Camera setup
        // ------------------------------
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());


        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                cameraMonitorViewId);


        pipeline = new BallDetectionPipeline();
        webcam.setPipeline(pipeline);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }


            @Override
            public void onError(int errorCode) {}
        });


        waitForStart();
        if (isStopRequested()) return;


        // ------------------------------
        // Ball-follow loop
        // ------------------------------
        while (opModeIsActive()) {


            telemetry.addData("Detected", pipeline.ballDetected);
            telemetry.addData("Color", pipeline.detectedColor);
            telemetry.addData("Object X", pipeline.ballX);
            telemetry.addData("Pixels", pipeline.pixelCount);
            telemetry.update();


            if (!pipeline.ballDetected) {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            } else {


                int frameCenter = 320 / 2;   // 160
                int error = pipeline.ballX - frameCenter;


                double forwardPower = 0.35;
                double turnPower = error / 200.0;


                turnPower = Math.max(-0.3, Math.min(0.3, turnPower));


                double leftPower  = forwardPower + turnPower;
                double rightPower = forwardPower - turnPower;


                leftDrive.setPower(leftPower);
                rightDrive.setPower(rightPower);
            }


            sleep(20);
        }


        leftDrive.setPower(0);
        rightDrive.setPower(0);
        webcam.stopStreaming();
    }


    // -----------------------------------------------------
    // Vision Pipeline (Contour-based, FTC-safe)
    // Detects ONLY GREEN and PURPLE objects (no red)
    // -----------------------------------------------------
    static class BallDetectionPipeline extends OpenCvPipeline {


        public boolean ballDetected = false;
        public double pixelCount = 0;
        public int ballX = -1;
        public String detectedColor = "NONE";


        private static final double MIN_PIXELS = 4000;


        @Override
        public Mat processFrame(Mat input) {


            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);


            // ---------------- GREEN ----------------
            Mat greenMask = new Mat();
            Scalar lowerGreen = new Scalar(35, 80, 80);
            Scalar upperGreen = new Scalar(85, 255, 255);
            Core.inRange(hsv, lowerGreen, upperGreen, greenMask);


            // ---------------- PURPLE ----------------
            Mat purpleMask = new Mat();
            Scalar lowerPurple = new Scalar(125, 80, 80);
            Scalar upperPurple = new Scalar(155, 255, 255);
            Core.inRange(hsv, lowerPurple, upperPurple, purpleMask);


            // Clean masks (reduce noise)
            cleanMask(greenMask);
            cleanMask(purpleMask);


            // Count pixels for each color
            double greenPixels  = Core.countNonZero(greenMask);
            double purplePixels = Core.countNonZero(purpleMask);


            // Choose the color with the most pixels
            Mat chosenMask;
            if (greenPixels >= purplePixels) {
                chosenMask = greenMask;
                pixelCount = greenPixels;
                detectedColor = "GREEN";
            } else {
                chosenMask = purpleMask;
                pixelCount = purplePixels;
                detectedColor = "PURPLE";
            }


            ballDetected = pixelCount > MIN_PIXELS;
            ballX = -1;


            if (ballDetected) {
                List<MatOfPoint> contours = new ArrayList<>();
                Mat hierarchy = new Mat();


                Imgproc.findContours(
                        chosenMask,
                        contours,
                        hierarchy,
                        Imgproc.RETR_EXTERNAL,
                        Imgproc.CHAIN_APPROX_SIMPLE
                );


                double maxArea = 0;
                Rect bestRect = null;


                for (MatOfPoint c : contours) {
                    double area = Imgproc.contourArea(c);
                    if (area > maxArea) {
                        maxArea = area;
                        bestRect = Imgproc.boundingRect(c);
                    }
                }


                if (bestRect != null) {
                    ballX = bestRect.x + bestRect.width / 2;


                    // Debug draw
                    Imgproc.rectangle(input, bestRect, new Scalar(0, 255, 0), 2);
                    Imgproc.putText(
                            input,
                            detectedColor,
                            new org.opencv.core.Point(bestRect.x, Math.max(0, bestRect.y - 5)),
                            Imgproc.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            new Scalar(0, 255, 0),
                            2
                    );
                }


                hierarchy.release();
            }


            // Cleanup
            hsv.release();
            greenMask.release();
            purpleMask.release();


            return input;
        }


        private void cleanMask(Mat mask) {
            Imgproc.GaussianBlur(mask, mask, new org.opencv.core.Size(5, 5), 0);
            Imgproc.erode(mask, mask, new Mat());
            Imgproc.dilate(mask, mask, new Mat());
        }
    }
}
