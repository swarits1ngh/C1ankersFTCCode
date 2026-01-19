package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

public class BallColorProcessor implements VisionProcessor {

    public volatile boolean ballDetected = false;

    private static final double MIN_AREA = 3000;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        Mat hsv = new Mat();
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);

        // GREEN mask
        Mat greenMask = new Mat();
        Core.inRange(hsv,
                new Scalar(35, 80, 80),
                new Scalar(85, 255, 255),
                greenMask);

        // BLUE mask
        Mat blueMask = new Mat();
        Core.inRange(hsv,
                new Scalar(95, 80, 80),
                new Scalar(125, 255, 255),
                blueMask);

        Mat combined = new Mat();
        Core.bitwise_or(greenMask, blueMask, combined);

        double area = Core.countNonZero(combined);
        ballDetected = area > MIN_AREA;

        // Debug overlay
        Imgproc.putText(
                frame,
                ballDetected ? "BALL DETECTED" : "NO BALL",
                new Point(10, 30),
                Imgproc.FONT_HERSHEY_SIMPLEX,
                0.8,
                ballDetected ? new Scalar(0, 255, 0) : new Scalar(0, 0, 255),
                2
        );

        hsv.release();
        greenMask.release();
        blueMask.release();
        combined.release();

        return null;
    }

    @Override
    public void onDrawFrame(
            Canvas canvas, int onscreenWidth, int onscreenHeight,
            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
            Object userContext) {}
}
