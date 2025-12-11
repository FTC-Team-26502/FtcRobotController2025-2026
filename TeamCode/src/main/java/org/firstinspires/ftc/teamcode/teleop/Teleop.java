package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class Teleop extends BaseCodeV2{


    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode(true,true,true,true,true);
        waitForStart();

        while (opModeIsActive()) {
            odo.update();
            shooting_okay();


//            telemetryOdometryUpdate();
            if(gamepad1.a) {
                intakeRun(1);
            } else if (gamepad1.b) {
                intakeRun(0);
            } else if (gamepad1.x && shooting_okay()) {
                shoot();
            } else if (gamepad1.y) {
                anglerLeft.setTargetPosition(0);
                anglerRight.setTargetPosition(0);
                stop_shooter();
            }
            driveSpeed(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
//            telemetry.addData("Green: ", color.green());
//            telemetry.addData("Blue: ", color.blue());
//            telemetry.addData("Red: ", color.red());
//            shooterLeft.setPower(0.5);
//            shooterRight.setPower(0.5);
            if (color.red() > 1500) {

                for (int i = 0; i < 1; i++) {


                    light.setPosition(LIGHTPURPLE);
                    sleep(500);
                    light.setPosition(0);

                }

            }

            if (color.green()>2000) {

                for (int i = 0; i < 1; i++) {

                    light.setPosition(LIGHTGREEN);
                    sleep(500);
                    light.setPosition(0);


                }
            }
            telemetry.addData("Motor Current in amps (left): ", shooterLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Motor Current in amps (right): ", shooterRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Target ticks;", anglerLeft.getTargetPosition());
            telemetry.addData("Current ticks;", anglerLeft.getCurrentPosition());
            telemetry.update();
        }
    }
}
