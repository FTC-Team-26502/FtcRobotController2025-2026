package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class testinghtisnkdjf extends BaseCodeV2{

    double Colors = LIGHTGREEN;
    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode(true,true,true,true,true);
        waitForStart();
        while (opModeIsActive()) {
            shooterLeft.setPower(1);
            shooterRight.setPower(1);
            telemetry.addData("Motor Current in amps (left): ", shooterLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Motor Current in amps (right): ", shooterRight.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}