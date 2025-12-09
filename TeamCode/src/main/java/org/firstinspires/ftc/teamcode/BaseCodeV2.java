package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.Locale;
import java.util.Map;

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
    protected Servo light;
    protected ColorSensor color;
    protected DistanceSensor flDistance;
    protected DistanceSensor blDistance;
    protected DistanceSensor frDistance;
    protected DistanceSensor brDistance;
    protected final double LIGHTRED = 0.3;
    protected final double LIGHTYELLOW = 0.388;
    protected final double LIGHTGREEN = 0.477;
    protected final double LIGHTBLUE = 0.611;
    protected final double LIGHTPURPLE = 0.722;

    protected double colors = LIGHTGREEN;
    protected final double MIN_SPEED_DRIVE = 0.2;
    protected final double ANGLER_SPEED = 0.05;
    protected final double DRIVE_SPEED_SCALE_DOWN = 1;
    protected final double ANGLE_TO_TICKS = 1;
    protected final long OUTTAKE_TIMEOUT = 300;
    protected final double DELTA_Y = 0.9; //meters
    protected final double GRAVITY = 9.80665; // m/s^2
    protected double speed= 0.5;
    protected final int ANGLE = 50;

    public void initOpMode(boolean drive, boolean odometry, boolean shooter, boolean intake, boolean sensors) {
        // Init Motors
        if(drive) {
            leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
            leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
            rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
            rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

            rightFront.setDirection(DcMotorEx.Direction.FORWARD);
            rightBack.setDirection(DcMotorEx.Direction.FORWARD);
            leftFront.setDirection(DcMotorEx.Direction.FORWARD);
            leftBack.setDirection(DcMotorEx.Direction.FORWARD);


            leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }
        //Init odometry
        if(odometry) {
            odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            odo.setOffsets(-82.0, -10.0, DistanceUnit.MM);
            odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        }
        //Init shooter
        if(shooter) {
            anglerLeft = hardwareMap.get(DcMotorEx.class, "anglerLeft");
            anglerRight = hardwareMap.get(DcMotorEx.class, "anglerRight");
            shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
            shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
            shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
            anglerLeft.setTargetPosition(0);
            anglerRight.setTargetPosition(0);
            anglerLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            anglerRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            anglerLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            anglerRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            anglerRight.setDirection(DcMotorEx.Direction.REVERSE);
            anglerLeft.setDirection(DcMotorEx.Direction.FORWARD);
        }
        // Init Servos
        if(intake) {
            inFrontLeft = hardwareMap.get(CRServo.class, "inFrontLeft");
            inFrontRight = hardwareMap.get(CRServo.class, "inFrontRight");
            inMiddleLeft = hardwareMap.get(CRServo.class, "inMiddleLeft");
            inMiddleRight = hardwareMap.get(CRServo.class, "inMiddleRight");
            inBackLeft = hardwareMap.get(CRServo.class, "inBackLeft");
            inBackRight = hardwareMap.get(CRServo.class, "inBackRight");

            inFrontLeft.setDirection(CRServo.Direction.REVERSE);
            inMiddleLeft.setDirection(CRServo.Direction.REVERSE);
            inBackLeft.setDirection(CRServo.Direction.REVERSE);
        }

        if(sensors){
            light = hardwareMap.get(Servo.class, "light");
            color = hardwareMap.get(ColorSensor.class, "color");
            flDistance = hardwareMap.get(DistanceSensor.class, "flDistance");
            blDistance = hardwareMap.get(DistanceSensor.class, "blDistance");
            frDistance = hardwareMap.get(DistanceSensor.class, "frDistance");
            brDistance = hardwareMap.get(DistanceSensor.class, "brDistance");
            light.setPosition(0.2);
        }
        telemetry.addLine("Init complete");
        telemetry.update();
    }

//     Run all the servos on the intake at input Power
    public void intakeRun(double power) {
        inFrontLeft.setPower(power);
        inFrontRight.setPower(power);
//        inBackLeft.setPower(power);
//        inBackRight.setPower(power);
        inMiddleLeft.setPower(power);
        inMiddleRight.setPower(power);
        telemetry.addData("Intake running at:", power);
    }

    public void stop_shooter() {

        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        anglerLeft.setPower(0);
        anglerRight.setPower(0);
        inBackLeft.setPower(0);
        inBackRight.setPower(0);

    }

    public void shoot() {
        // Spin both shooters
        speed = calculateShootingSpeed((blDistance.getDistance(DistanceUnit.METER)+brDistance.getDistance(DistanceUnit.METER))/2);
        shooterLeft.setPower(speed);
        shooterRight.setPower(speed); // fixed
        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        int ticks = (int) Math.round(ANGLE_TO_TICKS * ANGLE);
        anglerLeft.setPower(ANGLER_SPEED);
        anglerRight.setPower(ANGLER_SPEED);
        anglerLeft.setTargetPosition(ticks);
        anglerRight.setTargetPosition(ticks);
        while (anglerLeft.getPower()> 0.5) {
            if (anglerLeft.getCurrentPosition() < 42 & anglerLeft.getCurrentPosition() > 38) {
                anglerLeft.setTargetPosition(anglerLeft.getCurrentPosition());
                anglerRight.setTargetPosition(anglerRight.getCurrentPosition());
            }
            telemetry.addData("Power", anglerLeft.getPower());
            telemetry.addData("Target ticks;", anglerLeft.getTargetPosition());
            telemetry.addData("Current ticks;", anglerLeft.getCurrentPosition());
            telemetry.update();
        }
        sleep(2000);
        inBackLeft.setPower(1);
        inBackRight.setPower(1);
//
//        // Brief non-tight wait to spin up (avoid freezing loop)
//        long end = System.currentTimeMillis() + OUTTAKE_TIMEOUT;
//        while (System.currentTimeMillis() < end) {
//            telemetry.addLine("Speeding up Outtake");
//            telemetry.update();
//        }
    }

    public double calculateShootingSpeed(double d){
        speed = (d * Math.sqrt(GRAVITY/(d-DELTA_Y)))/55;
        telemetry.addData("D", d);
        telemetry.addData("SPEED", speed);
        telemetry.update();
        return speed;
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
        double lf = yPower + xPower - turnPower;
        double lb = yPower - xPower - turnPower;
        double rb = yPower + xPower + turnPower;
        double rf = yPower - xPower + turnPower;

        //Set motor powers
        leftFront.setPower(lf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
        rightFront.setPower(rf);
    }

//    public void telemetryOdometryUpdate() {
//
//        Pose2D pos = odo.getPosition();
////        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
//        telemetry.addData("Position", data);
//    }
        //TODO
//
//    telemetry.addData("Green: ", color.green());
//            telemetry.addData("Blue: ", color.blue());
//            telemetry.addData("Red: ", color.red());
//            light.setPosition(Colors);
//            if (color.green() > 1000) {
//        telemetry.addLine("Ball collected");
//        Colors = LIGHTPURPLE;
//        sleep(1000);
//
//
//    }
//
//            if (color.blue() > 1000) {
//        telemetry.addLine("Ball collected");
//        Colors = LIGHTPURPLE;
//        sleep(1000);
//
//
//
//
//    }
//
//            if (color.red() > 1000) {
//        telemetry.addLine("Ball collected");
//        Colors = LIGHTPURPLE;
//        sleep(1000);
//



    }

//}