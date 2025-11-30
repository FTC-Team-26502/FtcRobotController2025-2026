package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class NewTest extends LinearOpMode{


    private CRServo inFrontLeft;
    private CRServo inFrontRight;
    private CRServo inMiddleLeft;
    private CRServo inMiddleRight;
    private CRServo inBackLeft;
    private CRServo inBackRight;
    @Override
    public void runOpMode() throws InterruptedException {

        inFrontLeft = hardwareMap.get(CRServo.class, "inFrontLeft");
        inFrontRight = hardwareMap.get(CRServo.class, "inFrontRight");
        inMiddleLeft = hardwareMap.get(CRServo.class, "inMiddleLeft");   // fixed name
        inMiddleRight = hardwareMap.get(CRServo.class, "inMiddleRight");  // fixed name
        inBackLeft = hardwareMap.get(CRServo.class, "inBackLeft");
        inBackRight = hardwareMap.get(CRServo.class, "inBackRight");

        waitForStart();

        while (opModeIsActive()) {


            inFrontLeft.setPower(1);
            inFrontRight.setPower(1);
            inMiddleLeft.setPower(1);
            inMiddleRight.setPower(1);
            inBackLeft.setPower(1);
            inBackRight.setPower(1);


        }
    }
}
