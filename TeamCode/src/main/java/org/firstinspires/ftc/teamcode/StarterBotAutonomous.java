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
import org.opencv.core.Scalar;
import org.opencv.core.Core;
import org.opencv.imgproc.Imgproc;

@Autonomous(name = "StarterBot Ball Detect Auto (Improved)", group = "StarterBot")
public class StarterBotAutonomous extends LinearOpMode {

    private DcMotor leftDrive, rightDrive;
    private OpenCvCamera webcam;
    private BallDetectionPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Initialize drive motors ---
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // ✅ Correct directions so forward actually goes forward
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // --- Initialize webcam ---
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                cameraMonitorViewId);

        pipeline = new BallDetectionPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("Camera", "Opened Successfully");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        telemetry.addData("Status", "Initialized – Waiting for Start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("Ball Detected?", pipeline.ballDetected);
            telemetry.addData("Red Pixels", pipeline.pixelCount);
            telemetry.update();

            if (pipeline.ballDetected) {
                leftDrive.setPower(0.4);
                rightDrive.setPower(0.4);
            } else {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            }

            sleep(50); // small delay
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        webcam.stopStreaming();
    }

    // -----------------------------------------------------
    // Vision Pipeline (Improved)
    // -----------------------------------------------------
    static class BallDetectionPipeline extends OpenCvPipeline {

        public boolean ballDetected = false;
        public double pixelCount = 0;

        @Override
        public Mat processFrame(Mat input) {
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Define RED ranges (two regions in HSV space)
            Scalar lowerRed1 = new Scalar(0, 100, 100);
            Scalar upperRed1 = new Scalar(10, 255, 255);
            Scalar lowerRed2 = new Scalar(160, 100, 100);
            Scalar upperRed2 = new Scalar(179, 255, 255);

            Mat mask1 = new Mat();
            Mat mask2 = new Mat();
            Core.inRange(hsv, lowerRed1, upperRed1, mask1);
            Core.inRange(hsv, lowerRed2, upperRed2, mask2);

            Mat mask = new Mat();
            Core.addWeighted(mask1, 1.0, mask2, 1.0, 0.0, mask);

            // --- Noise reduction steps ---
            Imgproc.GaussianBlur(mask, mask, new org.opencv.core.Size(5, 5), 0);
            Imgproc.erode(mask, mask, new Mat());
            Imgproc.dilate(mask, mask, new Mat());

            // Count red pixels
            pixelCount = Core.countNonZero(mask);

            // Detection threshold
            ballDetected = pixelCount > 4000; // tune this if needed

            // Release memory
            hsv.release();
            mask1.release();
            mask2.release();
            mask.release();

            return input;
        }
    }
}
