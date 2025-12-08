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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Ball Detector (Purple + Green)")
public class WebcamBallDetector extends LinearOpMode {

    OpenCvCamera webcam;
    BallDetectionPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {

        int id = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), id);

        // Attach pipeline
        pipeline = new BallDetectionPipeline();
        webcam.setPipeline(pipeline);

        // Start webcam
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        waitForStart();

        while(opModeIsActive()) {

            telemetry.addData("Green Pixels", pipeline.greenCount);
            telemetry.addData("Purple Pixels", pipeline.purpleCount);

            telemetry.addData("Green Ball",
                    (pipeline.greenCount > 600) ? "DETECTED" : "Not seen");

            telemetry.addData("Purple Ball",
                    (pipeline.purpleCount > 600) ? "DETECTED" : "Not seen");

            telemetry.update();

            sleep(20);
        }
    }
}