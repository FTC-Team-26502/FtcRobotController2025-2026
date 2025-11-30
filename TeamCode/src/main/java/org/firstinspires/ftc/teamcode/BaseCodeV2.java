package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

public abstract class BaseCodeV2 extends LinearOpMode {

    protected GoBildaPinpointDriver odo;
    protected DcMotorEx leftFront;
    protected DcMotorEx leftBack;
    protected DcMotorEx rightFront;
    protected DcMotorEx rightBack;
    protected DcMotorEx anglerLeft;
    protected DcMotorEx anglerRight;
    protected DcMotorEx shooterLeft;
    protected DcMotorEx shooterRight;
    protected CRServo inFrontLeft;
    protected CRServo inFrontRight;
    protected CRServo inMiddleLeft;
    protected CRServo inMiddleRight;
    protected CRServo inBackLeft;
    protected CRServo inBackRight;
    protected final double MIN_SPEED_DRIVE = 0.2;
    protected final double ANGLER_SPEED = 0.5;
    protected final double DRIVE_SPEED_SCALE_DOWN = 0.5;
    protected final double ANGLE_TO_TICKS = 537.7/360;
    protected final long OUTTAKE_TIMEOUT = 300;

    public void initOpMode() {
        // Init Motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        odo.setOffsets(-82.0, -10.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

//
//        anglerLeft = hardwareMap.get(DcMotorEx.class, "anglerLeft");
//        anglerRight = hardwareMap.get(DcMotorEx.class, "anglerRight");
//        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
//        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        // Init Servos
//        inFrontLeft = hardwareMap.get(CRServo.class, "inFrontLeft");
//        inFrontRight = hardwareMap.get(CRServo.class, "inFrontRight");
//        inMiddleLeft = hardwareMap.get(CRServo.class, "inMiddleLeft");   // fixed name
//        inMiddleRight = hardwareMap.get(CRServo.class, "inMiddleRight");  // fixed name
//        inBackLeft = hardwareMap.get(CRServo.class, "inBackLeft");
//        inBackRight = hardwareMap.get(CRServo.class, "inBackRight");
//        bumperLeft = hardwareMap.get(Servo.class, "bumperLeft");
//        bumperRight = hardwareMap.get(Servo.class, "bumperRight");

        // Drive directions/behaviors (adjust if your wiring is different)
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);



        // CRServo directions (use CRServo.Direction)
//        inFrontRight.setDirection(CRServo.Direction.REVERSE);
//        inMiddleRight.setDirection(CRServo.Direction.REVERSE);
//        inBackRight.setDirection(CRServo.Direction.REVERSE);
//
//        // Prepare angler to run to position later
//        anglerLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        anglerRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        anglerLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        anglerRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        anglerRight.setDirection(DcMotorExSimple.Direction.REVERSE);
//        anglerLeft.setDirection(DcMotorExSimple.Direction.FORWARD);

        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);

        telemetry.addLine("Init complete");
        telemetry.update();
    }

    // Run all the servos on the intake at input Power
//    public void intakeRun(double power) {
//        inFrontLeft.setPower(power);
//        inFrontRight.setPower(power);
//        inBackLeft.setPower(power);
//        inBackRight.setPower(power);
//        inMiddleLeft.setPower(power);
//        inMiddleRight.setPower(power);
//        telemetry.addData("Intake running at:", power);
//    }
//
//    public void shoot(int angle, double speed) {
//        // Spin both shooters
//        shooterLeft.setPower(speed);
//        shooterRight.setPower(speed); // fixed
//
//        int ticks = (int) Math.round(ANGLE_TO_TICKS * angle);
//        // Move angler (requires RUN_TO_POSITION and power)
//        anglerLeft.setTargetPosition(ticks);
//        anglerRight.setTargetPosition(ticks);
//        anglerLeft.setPower(ANGLER_SPEED);
//        anglerRight.setPower(ANGLER_SPEED);
//
//        // Brief non-tight wait to spin up (avoid freezing loop)
//        long end = System.currentTimeMillis() + OUTTAKE_TIMEOUT;
//        while (opModeIsActive() && System.currentTimeMillis() < end) {
//            telemetry.addLine("Speeding up Outtake");
//            telemetry.update();
//            idle();
//        }
//
//        // Bumper fire and reset with a short delay
//        bumperLeft.setPosition(BUMPING_POSITION);
//        bumperRight.setPosition(BUMPING_POSITION);
//        safeSleep(120);
//        bumperLeft.setPosition(RESTING_POSITION);
//        bumperRight.setPosition(RESTING_POSITION);
//    }

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