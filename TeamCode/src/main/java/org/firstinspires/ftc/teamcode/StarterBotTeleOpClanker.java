package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="StarterBot Joystick + Flywheel + Servos", group="StarterBot")
private class StarterBotTeleOpClanker extends LinearOpMode {

    private DcMotor leftDrive, rightDrive;
    private DcMotor mainFlywhee
    private Servo rightSmallWheel, leftSmallWheel;

    @Override
    public void runOpMode() {

        // Drive motors
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Additional motor and servos
        mainFlywheel = hardwareMap.get(DcMotor.class, "main_flywheel");
        rightSmallWheel = hardwareMap.get(Servo.class, "right_small_wheel");
        leftSmallWheel = hardwareMap.get(Servo.class, "left_small_wheel");

        // Set motor directions
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        mainFlywheel.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // --- Drive control ---
            double move = -gamepad1.left_stick_y;   // Forward/backward
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

            // --- Flywheel control (counterclockwise) ---
            double flywheelPower = gamepad1.right_trigger;
            mainFlywheel.setPower(-flywheelPower);

            // --- Servo control for small wheels ---
            double servoInput = gamepad1.left_trigger; // 0 to 1
            // Convert trigger pressure to servo positions (adjust range as needed)
            double leftServoPos = 0.5 + (servoInput * 0.5);   // moves clockwise
            double rightServoPos = 0.5 - (servoInput * 0.5);  // moves counterclockwise

            leftSmallWheel.setPosition(leftServoPos);
            rightSmallWheel.setPosition(rightServoPos);

            // --- Telemetry ---
            telemetry.addData("Drive", "Move=%.2f Turn=%.2f", move, turn);
            telemetry.addData("Left Power", "%.2f", leftPower);
            telemetry.addData("Right Power", "%.2f", rightPower);
            telemetry.addData("Flywheel Power", "%.2f", flywheelPower);
            telemetry.addData("Left Servo Pos", "%.2f", leftServoPos);
            telemetry.addData("Right Servo Pos", "%.2f", rightServoPos);
            telemetry.update();
        }
    }
}
