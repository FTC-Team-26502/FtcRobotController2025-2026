package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class IntakeTestTele extends LinearOpMode {
    private CRServo intakeServo;
    private CRServo intakeServo1;
    private CRServo intakeServo2;
    private CRServo intakeServo3;
    private CRServo intakeServo4;
    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;
    private double power;
    @Override
    public void runOpMode() throws InterruptedException {
            leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
            leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
            rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
            rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
            intakeServo = hardwareMap.get(CRServo.class, "servo");
            intakeServo1 = hardwareMap.get(CRServo.class, "servo1");
            intakeServo2 = hardwareMap.get(CRServo.class, "servo2");
        intakeServo3 = hardwareMap.get(CRServo.class, "servo3");
        intakeServo4 = hardwareMap.get(CRServo.class, "servo4");
            waitForStart();
            while(opModeIsActive()) {
                 power = gamepad1.left_stick_y;
                 leftFront.setPower(-power);
                 leftBack.setPower(-power);
                 rightBack.setPower(power);
                 rightFront.setPower(power);

                 if (gamepad1.a) {
                     intakeServo.setPower(-1);
                     intakeServo1.setPower(-1);
                     intakeServo2.setPower(1);
                     intakeServo3.setPower(1);
                     intakeServo4.setPower(1);
                 } else if (gamepad1.b) {
                     intakeServo.setPower(0);
                     intakeServo1.setPower(0);
                     intakeServo2.setPower(0);
                     intakeServo3.setPower(0);
                     intakeServo4.setPower(0);
                 } else if (gamepad1.x) {
                     intakeServo.setPower(1);
                     intakeServo1.setPower(1);
                     intakeServo2.setPower(-1);
                     intakeServo3.setPower(-1);
                     intakeServo4.setPower(-1);
                 }
             }


    }
}
