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
public class testdrive extends LinearOpMode{

    private DcMotorEx leftFront;
    private DcMotorEx leftBack;

    private DcMotorEx rightFront;

    private DcMotorEx rightBack;
    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        waitForStart();

        while (opModeIsActive()) {

            leftFront.setPower(0.2);
            leftBack.setPower(0.2);
            rightFront.setPower(0.2);
            rightBack.setPower(0.2);

            telemetry.addData("Motor Currents leftFront(AMPS)", leftFront.getCurrent(CurrentUnit.AMPS));


            telemetry.addData("Motor Currents leftBack(AMPS)", leftBack.getCurrent(CurrentUnit.AMPS));


            telemetry.addData("Motor Currents rightFront(AMPS)", rightFront.getCurrent(CurrentUnit.AMPS));

            telemetry.addData("Motor Currents rightBack(AMPS)", rightBack.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
