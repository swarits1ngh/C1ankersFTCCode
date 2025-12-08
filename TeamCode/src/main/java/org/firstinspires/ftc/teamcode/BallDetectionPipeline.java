int cameraMonitorViewId = hardwareMap
        .appContext
        .getResources()
        .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

webcam = OpenCvCameraFactory.getInstance().createWebcam(
        hardwareMap.get(WebcamName.class, "Webcam 1"),
cameraMonitorViewId
);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
    @Override
    public void onOpened() {
        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
    }

    @Override
    public void onError(int errorCode) {}
});
        package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BallDetectionPipeline extends OpenCvPipeline {

    public int greenCount = 0;
    public int purpleCount = 0;

    @Override
    public Mat processFrame(Mat input) {

        // Convert RGB → HSV
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // GREEN HSV range
        Scalar lowGreen = new Scalar(40, 60, 60);
        Scalar highGreen = new Scalar(85, 255, 255);

        // PURPLE HSV range
        Scalar lowPurple = new Scalar(130, 60, 60);
        Scalar highPurple = new Scalar(165, 255, 255);

        // Generate masks
        Mat greenMask = new Mat();
        Core.inRange(hsv, lowGreen, highGreen, greenMask);

        Mat purpleMask = new Mat();
        Core.inRange(hsv, lowPurple, highPurple, purpleMask);

        // Count matching pixels
        greenCount = Core.countNonZero(greenMask);
        purpleCount = Core.countNonZero(purpleMask);

        // Combine masks for output display
        Mat output = new Mat();
        Core.addWeighted(greenMask, 1, purpleMask, 1, 0, output);

        Imgproc.cvtColor(output, output, Imgproc.COLOR_GRAY2RGB);

        return output; // shows mask view on Driver Hub
    }
}