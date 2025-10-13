package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test OpMode with Triggers", group = "Test")
public class testClass extends baseCode {
    private boolean intakeRunning = false; // Tracks if the intake is running
    private boolean rightTriggerPressed = false; // Tracks the right trigger state

    @Override
    public void runOpMode() {
        initOpMode(); // Ensure this is implemented in baseCode or testClass

        waitForStart();

        boolean leftTriggerPressed = false; // Tracks the left trigger state

        while (opModeIsActive()) {
            // Check if the left trigger is pressed for shooting
            if (gamepad1.left_trigger > 0.5 && !leftTriggerPressed) { // Threshold for trigger press
                leftTriggerPressed = true;
                shoot(45, 1); // Ensure this is implemented in baseCode
            } else if (gamepad1.left_trigger <= 0.5) { // Reset when the trigger is released
                leftTriggerPressed = false;
            }

            // Toggle intake on and off when the right trigger is pressed
            if (gamepad1.right_trigger > 0.5 && !rightTriggerPressed) { // Threshold for trigger press
                rightTriggerPressed = true; // Prevent multiple triggers
                intakeRunning = !intakeRunning; // Toggle the intake state
                if (intakeRunning) {
                    intakeRun(1); // Start the intake
                } else {
                    intakeRun(0); // Stop the intake
                }
            } else if (gamepad1.right_trigger <= 0.5) { // Reset when the trigger is released
                rightTriggerPressed = false;
            }

            // Drive logic
            driveSpeed(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x); // Ensure this is implemented in baseCode

            // Update telemetry
            telemetry.addData("Intake Running", intakeRunning);
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
