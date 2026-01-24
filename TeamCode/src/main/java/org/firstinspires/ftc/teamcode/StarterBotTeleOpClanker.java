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
    private DcMotor intakeBaby;
    private Servo rightSmallWheel, leftSmallWheel;

    @Override
    public void runOpMode() {

        // ---------------- HARDWARE MAP ----------------
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        mainFlywheel = hardwareMap.get(DcMotor.class, "main_flywheel");
        intakeBaby   = hardwareMap.get(DcMotor.class, "intake_baby");

        rightSmallWheel = hardwareMap.get(Servo.class, "right_small_wheel");
        leftSmallWheel  = hardwareMap.get(Servo.class, "left_small_wheel");

        // ---------------- MOTOR SETUP ----------------
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeBaby.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Drive motors direction (ensure forward is forward)
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        mainFlywheel.setDirection(DcMotor.Direction.FORWARD);
        intakeBaby.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mainFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeBaby.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ================= MAIN LOOP =================
        while (opModeIsActive()) {

            // ---------------- DRIVE CONTROL ----------------
            // Left stick UP/DOWN = forward/back
            // Right stick LEFT/RIGHT = turn
            double move = -gamepad1.left_stick_y;    // Forward/backward
            double turn = -gamepad1.right_stick_x;   // Corrected: RIGHT = turn right

            double leftPower  = move - turn;
            double rightPower = move + turn;

            // Normalize motor power
            double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > 1.0) {
                leftPower /= max;
                rightPower /= max;
            }

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // ---------------- FLYWHEEL ----------------
            double flywheelPower = gamepad1.right_trigger * 0.65;
            mainFlywheel.setPower(flywheelPower);

            // ---------------- INTAKE ----------------
            if (gamepad1.left_bumper) {
                intakeBaby.setPower(-1.0);
            } else {
                intakeBaby.setPower(0.0);
            }

            // ---------------- SERVO CONTROL (REVERSED) ----------------
            double servoInput = gamepad1.left_trigger; // 0..1
            double leftServoPos  = 0.5 - (servoInput * 0.5);
            double rightServoPos = 0.5 + (servoInput * 0.5);

            leftSmallWheel.setPosition(leftServoPos);
            rightSmallWheel.setPosition(rightServoPos);

            // ---------------- TELEMETRY ----------------
            telemetry.addData("Move (Left Y)", "%.2f", move);
            telemetry.addData("Turn (Right X)", "%.2f", turn);
            telemetry.addData("Left Power", "%.2f", leftPower);
            telemetry.addData("Right Power", "%.2f", rightPower);
            telemetry.addData("Flywheel", "%.2f", flywheelPower);
            telemetry.addData("Intake", gamepad1.left_bumper ? "ON" : "OFF");
            telemetry.addData("Left Servo", "%.2f", leftServoPos);
            telemetry.addData("Right Servo", "%.2f", rightServoPos);
            telemetry.update();
        }
    }
}
