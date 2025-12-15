// StarterBotTeleOpClanker.java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="StarterBot Joystick + Flywheel + Servos + Intake", group="StarterBot")
public class StarterBotTeleOpClanker extends LinearOpMode {

    private DcMotor leftDrive, rightDrive;
    private DcMotor mainFlywheel;
    private DcMotor intakeBaby; // NEW: intake motor
    private Servo rightSmallWheel, leftSmallWheel;

    @Override
    public void runOpMode() {

        // Drive motors
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Additional motor and servos
        mainFlywheel = hardwareMap.get(DcMotor.class, "main_flywheel");
        intakeBaby   = hardwareMap.get(DcMotor.class, "intake_baby"); // NEW
        rightSmallWheel = hardwareMap.get(Servo.class, "right_small_wheel");
        leftSmallWheel  = hardwareMap.get(Servo.class, "left_small_wheel");

        // Ensure consistent motor behavior
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeBaby.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set motor directions so positive power drives robot forward
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Flywheel direction (adjust if needed)
        mainFlywheel.setDirection(DcMotor.Direction.FORWARD);

        // Intake direction: set FORWARD so positive power spins clockwise (adjust if your mechanism needs the opposite)
        intakeBaby.setDirection(DcMotor.Direction.FORWARD);

        // Optional: set run modes
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mainFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeBaby.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // --- Drive control ---
            double move = -gamepad1.left_stick_y;   // Forward/backward (invert: up is negative)
            double turn = gamepad1.right_stick_x;   // Turning

            double leftPower  = move + turn;
            double rightPower = move - turn;

            // Normalize powers
            double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > 1.0) {
                leftPower /= max;
                rightPower /= max;
            }

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // --- Flywheel control ---
            double flywheelMaxPercent = 0.65;  // 75% cap
            double flywheelPower = gamepad1.right_trigger * flywheelMaxPercent;
            mainFlywheel.setPower(flywheelPower);


            // --- Intake control ---
            // PS4 R1 (left shoulder in your mapping) -> gamepad1.left_bumper
            // Press to spin clockwise at 100%, release to stop.
            if (gamepad1.left_bumper) {
                intakeBaby.setPower(-1.0); // full speed clockwise
            } else {
                intakeBaby.setPower(0.0); // stop
            }

            // If your intake spins the wrong way, flip direction or set power to -1.0
            // intakeBaby.setDirection(DcMotor.Direction.REVERSE);
            // or:
            // intakeBaby.setPower(-1.0);

            // --- Servo control for small wheels ---
            double servoInput = gamepad1.left_trigger; // 0..1
            double leftServoPos = 0.5 + (servoInput * 0.5);   // moves clockwise
            double rightServoPos = 0.5 - (servoInput * 0.5);  // moves counterclockwise

            leftSmallWheel.setPosition(leftServoPos);
            rightSmallWheel.setPosition(rightServoPos);

            // --- Telemetry ---
            telemetry.addData("Drive", "Move=%.2f Turn=%.2f", move, turn);
            telemetry.addData("Left Power", "%.2f", leftPower);
            telemetry.addData("Right Power", "%.2f", rightPower);
            telemetry.addData("Flywheel Power", "%.2f", flywheelPower);
            telemetry.addData("Intake", gamepad1.left_bumper ? "ON (1.0)" : "OFF (0.0)");
            telemetry.addData("Left Servo Pos", "%.2f", leftServoPos);
            telemetry.addData("Right Servo Pos", "%.2f", rightServoPos);
            telemetry.update();
        }
    }
}
