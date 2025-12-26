package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp(name = "Motor test")
public abstract class motorTest extends LinearOpMode {
    double power = 0.2;
    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;

    @Override
    public void runOpMode() throws InterruptedException {
        shooterLeft = hardwareMap.get(DcMotorEx.class, "left");
        shooterRight = hardwareMap.get(DcMotorEx.class, "right");
        waitForStart();

        while(opModeIsActive()) {


            if (gamepad1.a) {

                power += 0.1;

            }

            if (gamepad1.b) {

                power -= 0.1;

            }

            shooterRight.setPower(power);
            shooterLeft.setPower(power);

            telemetry.addData("Motor Current in amps (left): ", shooterLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.update();

            telemetry.addData("Motor Current in amps (right): ", shooterRight.getCurrent(CurrentUnit.AMPS));
            telemetry.update();

        }

    }
}
