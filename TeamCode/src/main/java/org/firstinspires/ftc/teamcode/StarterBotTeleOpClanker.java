package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="StarterBot Buttons + Flywheel + Servos + Intake", group="StarterBot")
public class StarterBotTeleOpClanker extends LinearOpMode {

    private DcMotor leftDrive, rightDrive;
    private DcMotor mainFlywheel;
    private DcMotor intakeBaby;
    private Servo rightSmallWheel, leftSmallWheel;

    // Flywheel toggle state
    private boolean flywheelOn = false;
    private boolean lastAState = false;

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

        // Motor directions
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
            double move = -gamepad1.left_stick_y;   // Forward/back
            double turn = gamepad1.right_stick_x;   // FIXED: right = turn right

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

            // ---------------- FLYWHEEL (A BUTTON TOGGLE) ----------------
            boolean currentAState = gamepad1.a;
            if (currentAState && !lastAState) {
                flywheelOn = !flywheelOn;
            }
            lastAState = currentAState;

            mainFlywheel.setPower(flywheelOn ? 0.65 : 0.0);

            // ---------------- INTAKE ----------------
            if (gamepad1.left_bumper) {
                intakeBaby.setPower(-1.0);
            } else {
                intakeBaby.setPower(0.0);
            }

            // ---------------- SERVO CONTROL (BUTTONS) ----------------
            // X = OPEN, Y = CLOSE
            if (gamepad1.x) {
                leftSmallWheel.setPosition(0.0);
                rightSmallWheel.setPosition(1.0);
            }
            else if (gamepad1.y) {
                leftSmallWheel.setPosition(0.5);
                rightSmallWheel.setPosition(0.5);
            }

            // ---------------- TELEMETRY ----------------
            telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
            telemetry.addData("Left Power", "%.2f", leftPower);
            telemetry.addData("Right Power", "%.2f", rightPower);
            telemetry.addData("Left Servo", "%.2f", leftSmallWheel.getPosition());
            telemetry.addData("Right Servo", "%.2f", rightSmallWheel.getPosition());
            telemetry.update();
        }
    }
}
