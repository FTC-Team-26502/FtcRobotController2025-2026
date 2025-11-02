package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class baseCodeV2 extends LinearOpMode {
    protected DcMotor leftFront;
    protected DcMotor leftBack;
    protected DcMotor rightFront;
    protected DcMotor rightBack;
    protected DcMotor anglerLeft;
    protected DcMotor anglerRight;
    protected DcMotor shooterLeft;
    protected DcMotor shooterRight;
    protected CRServo inFrontLeft;
    protected CRServo inFrontRight;
    protected CRServo inMiddleLeft;
    protected CRServo inMiddleRight;
    protected CRServo inBackLeft;
    protected CRServo inBackRight;
    protected Servo bumperLeft;
    protected Servo bumperRight;
    protected final double BUMPING_POSITION = 1;
    protected final double RESTING_POSITION = 0;
    protected final double MIN_SPEED_DRIVE = 0.2;
    protected final double ANGLER_SPEED = 0.5;
    protected final double DRIVE_SPEED_SCALE_DOWN = 0.5;
    protected final double ANGLE_TO_TICKS = 537.7/360;
    protected final long OUTTAKE_TIMEOUT = 300;

    public void initOpMode() {
        // Init Motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        anglerLeft = hardwareMap.get(DcMotor.class, "anglerLeft");
        anglerRight = hardwareMap.get(DcMotor.class, "anglerRight");
        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");

        // Init Servos
        inFrontLeft = hardwareMap.get(CRServo.class, "inFrontLeft");
        inFrontRight = hardwareMap.get(CRServo.class, "inFrontRight");
        inMiddleLeft = hardwareMap.get(CRServo.class, "inMiddleLeft");   // fixed name
        inMiddleRight = hardwareMap.get(CRServo.class, "inMiddleRight");  // fixed name
        inBackLeft = hardwareMap.get(CRServo.class, "inBackLeft");
        inBackRight = hardwareMap.get(CRServo.class, "inBackRight");
        bumperLeft = hardwareMap.get(Servo.class, "bumperLeft");
        bumperRight = hardwareMap.get(Servo.class, "bumperRight");

        // Drive directions/behaviors (adjust if your wiring is different)
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // CRServo directions (use CRServo.Direction)
        inFrontRight.setDirection(CRServo.Direction.REVERSE);
        inMiddleRight.setDirection(CRServo.Direction.REVERSE);
        inBackRight.setDirection(CRServo.Direction.REVERSE);

        // Prepare angler to run to position later
        anglerLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        anglerRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        anglerLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        anglerRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        anglerRight.setDirection(DcMotorSimple.Direction.REVERSE);
        anglerLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("Init complete");
        telemetry.update();
    }

    // Run all the servos on the intake at input Power
    public void intakeRun(double power) {
        inFrontLeft.setPower(power);
        inFrontRight.setPower(power);
        inBackLeft.setPower(power);
        inBackRight.setPower(power);
        inMiddleLeft.setPower(power);
        inMiddleRight.setPower(power);
        telemetry.addData("Intake running at:", power);
    }

    public void shoot(int angle, double speed) {
        // Spin both shooters
        shooterLeft.setPower(speed);
        shooterRight.setPower(speed); // fixed

        int ticks = (int) Math.round(ANGLE_TO_TICKS * angle);
        // Move angler (requires RUN_TO_POSITION and power)
        anglerLeft.setTargetPosition(ticks);
        anglerRight.setTargetPosition(ticks);
        anglerLeft.setPower(ANGLER_SPEED);
        anglerRight.setPower(ANGLER_SPEED);

        // Brief non-tight wait to spin up (avoid freezing loop)
        long end = System.currentTimeMillis() + OUTTAKE_TIMEOUT;
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            telemetry.addLine("Speeding up Outtake");
            telemetry.update();
            idle();
        }

        // Bumper fire and reset with a short delay
        bumperLeft.setPosition(BUMPING_POSITION);
        bumperRight.setPosition(BUMPING_POSITION);
        safeSleep(120);
        bumperLeft.setPosition(RESTING_POSITION);
        bumperRight.setPosition(RESTING_POSITION);
    }

    public void driveSpeed(double xPower, double yPower, double turnPower) {
        // Scale down
        xPower *= DRIVE_SPEED_SCALE_DOWN;
        yPower *= DRIVE_SPEED_SCALE_DOWN;
        turnPower *= DRIVE_SPEED_SCALE_DOWN;

        // Deadzones on absolute value
        if (Math.abs(xPower) < MIN_SPEED_DRIVE) xPower = 0;
        if (Math.abs(yPower) < MIN_SPEED_DRIVE) yPower = 0;
        if (Math.abs(turnPower) < MIN_SPEED_DRIVE) turnPower = 0;

        // Mecanum mix (robot-centric)
        double lf = yPower - xPower - turnPower;
        double lb = yPower + xPower - turnPower;
        double rb = yPower - xPower + turnPower;
        double rf = yPower + xPower + turnPower;

        //Set motor powers
        leftFront.setPower(lf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
        rightFront.setPower(rf);
    }

    // Helper to avoid tight blocking
    private void safeSleep(long ms) {
        long end = System.currentTimeMillis() + ms;
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            idle();
        }
    }
}