package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.robotcore.hardware.Servo;

import java.util.Locale;

@TeleOp
public class testdrive extends BaseCodeV2{


    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode(true, true, false, false , true);
        waitForStart();

        while (opModeIsActive()) {
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Color", color.green());
            telemetry.addData("Position", data);
            telemetry.update();
//            leftFront.setPower(0.2);
//            leftBack.setPower(0.2);
//            rightFront.setPower(0.2);
//            rightBack.setPower(0.2);
//
//            telemetry.addData("Motor Currents leftFront(AMPS)", leftFront.getCurrent(CurrentUnit.AMPS));

//
//            telemetry.addData("Motor Currents leftBack(AMPS)", leftBack.getCurrent(CurrentUnit.AMPS));
//
//
//            telemetry.addData("Motor Currents rightFront(AMPS)", rightFront.getCurrent(CurrentUnit.AMPS));
//
//            telemetry.addData("Motor Currents rightBack(AMPS)", rightBack.getCurrent(CurrentUnit.AMPS));
//            telemetry.update();

//            inFrontLeft.setPower(-1);
//            inFrontRight.setPower(1);
//            inMiddleLeft.setPower(-0.1);
//            inMiddleRight.setPower(0.1);
//            inBackLeft.setPower(-0.1);
//            inBackRight.setPower(0.1);


            
        }
    }
}
