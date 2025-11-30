package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
public abstract class BaseCode extends LinearOpMode {
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
    protected CRServo inBackLeft;
    protected CRServo inBackRight;
    protected Servo bumperLeft;
    protected Servo bumperRight;
//    protected ColorSensor ballColor1;
//    protected ColorSensor ballColor2;
//    protected ColorSensor ballColor3;
    protected final double BUMPIND_POSITION = 1;
    protected final double RESTING_POSITION = 0;
    protected final double MIN_SPEED_DRIVE = 0.2;
    public void initOpMode() {
        //Init Motors
        leftFront    = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack     = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront   = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack    = hardwareMap.get(DcMotor.class, "rightBack");

        anglerLeft   = hardwareMap.get(DcMotor.class, "anglerLeft");
        anglerRight  = hardwareMap.get(DcMotor.class, "anglerRight");
        shooterLeft  = hardwareMap.get(DcMotor.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");

        //Init Servos
        inFrontLeft  = hardwareMap.get(CRServo.class, "inFrontLeft");
        inFrontRight = hardwareMap.get(CRServo.class, "inFrontRight");
        inBackLeft   = hardwareMap.get(CRServo.class, "inBackLeft");
        inBackRight  = hardwareMap.get(CRServo.class, "inBackRight");
        bumperLeft   = hardwareMap.get(Servo.class, "bumperLeft");
        bumperRight  = hardwareMap.get(Servo.class, "bumperRight");

        //Init Sensors
//        ballColor1   = hardwareMap.get(ColorSensor.class, "ballColor1");
//        ballColor2   = hardwareMap.get(ColorSensor.class, "ballColor2");
//        ballColor3   = hardwareMap.get(ColorSensor.class, "ballColor3");

        //Add directions for motors

    }

    public void intakeRun(double power){
        inFrontLeft.setPower(power);
        inFrontRight.setPower(-power);
        inBackLeft.setPower(power);
        inBackRight.setPower(-power);
    }

    public void shoot(int angle, double speed){
        shooterLeft.setPower(speed);
        shooterLeft.setPower(speed);
        anglerLeft.setTargetPosition(angle);
        anglerRight.setTargetPosition(-angle);
        while (shooterLeft.getPower()+shooterRight.getPower() < 2*speed){
            telemetry.addLine("Speeding up Outtake");
        }
        bumperLeft.setPosition(BUMPIND_POSITION);
        bumperLeft.setPosition(RESTING_POSITION);
    }

    public void driveSpeed(double xPower, double yPower, double turnPower){
        xPower *= 0.5;
        yPower *= 0.5;
        turnPower *= 0.5;
        if (xPower < MIN_SPEED_DRIVE){
            xPower = 0;
        }
        if (yPower < MIN_SPEED_DRIVE){
            yPower = 0;
        }
        if (turnPower < MIN_SPEED_DRIVE){                     
            turnPower = 0;
        }
        leftFront.setPower(yPower-xPower-turnPower);
        leftBack.setPower(yPower+xPower-turnPower);
        rightBack.setPower(xPower-yPower+turnPower);
        rightFront.setPower(-yPower-xPower+turnPower);
    }

}
