package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.util.RobotLog;


@TeleOp(name = "Motor Measurer", group = "Test")
public abstract class CurrentMeasurer extends LinearOpMode {
    double power = 0.2;
    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;

    @Override
    public void runOpMode(){
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        waitForStart();

        while(opModeIsActive()) {


            if (gamepad1.a) {

                power += 0.1;

            }

            if (gamepad1.b) {

                power -= 0.1;

            }

            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(power);

            telemetry.addData("Motor Current in amps (leftFront): ", leftFront.getCurrent(CurrentUnit.AMPS));
            telemetry.update();

            telemetry.addData("Motor Current in amps (leftBack): ", leftBack.getCurrent(CurrentUnit.AMPS));
            telemetry.update();

            telemetry.addData("Motor Current in amps (rightFront): ", rightFront.getCurrent(CurrentUnit.AMPS));
            telemetry.update();

            telemetry.addData("Motor Current in amps (rightBack): ", rightBack.getCurrent(CurrentUnit.AMPS));
            telemetry.update();

        }

    }
}
