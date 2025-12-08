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

@TeleOp(name = "HSV Calibration Tool")
public class HSVCalibrationOpMode extends LinearOpMode {

    OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {

        int id = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), id);

        // SHOW RAW CAMERA FEED (NO PIPELINE)
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        telemetry.addLine("Point the camera at the PURPLE or GREEN ball.");
        telemetry.addLine("Then use Driver Hub color picker to read HSV.");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            sleep(50);
        }
    }
}