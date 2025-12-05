package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class Teleop extends BaseCodeV2{

    double Colors = LIGHTGREEN;
    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode(true,true,true,true,true);
        waitForStart();

        while (opModeIsActive()) {
            odo.update();
            telemetryOdometryUpdate();
            if(gamepad1.a) {
                intakeRun(1);
            } else if (gamepad1.b) {
                intakeRun(0);
            } else if (gamepad1.x) {
                shoot(35, 1);
            } else if (gamepad1.y) {
                anglerLeft.setTargetPosition(0);
                anglerRight.setTargetPosition(0);
                stop_shooter();
            }
            driveSpeed(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
//            telemetry.addData("Green: ", color.green());
//            telemetry.addData("Blue: ", color.blue());
//            telemetry.addData("Red: ", color.red());

            telemetry.update();
        }
    }
}
