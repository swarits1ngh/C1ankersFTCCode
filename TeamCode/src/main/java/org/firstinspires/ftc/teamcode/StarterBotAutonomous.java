package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "StarterBot Auto (Tag + Ball)", group = "StarterBot")
public class StarterBotAutonomous extends LinearOpMode {

    private DcMotor leftDrive, rightDrive;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private BallColorProcessor ballProcessor;
    private AprilTagVision tagVision;

    @Override
    public void runOpMode() {

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        ballProcessor = new BallColorProcessor();

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                aprilTag,
                ballProcessor
        );

        tagVision = new AprilTagVision(aprilTag);

        telemetry.addLine("Init complete");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            tagVision.update();

            // 🚗 MOVE WHEN BALL SEEN
            if (ballProcessor.ballDetected) {
                leftDrive.setPower(0.3);
                rightDrive.setPower(0.3);
            } else {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            }

            telemetry.addData("Ball Detected", ballProcessor.ballDetected);
            telemetry.addData("Last Tag ID", tagVision.lastSeenTagId);
            telemetry.addData("Last Goal", tagVision.lastSeenGoal);
            telemetry.addData("First Obelisk", tagVision.firstSeenPattern);
            telemetry.addData("Current Pattern", tagVision.currentPattern);
            telemetry.update();

            idle();
        }

        visionPortal.close();
    }
}
