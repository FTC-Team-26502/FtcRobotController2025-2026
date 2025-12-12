package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.BaseCodeV3;

public abstract class Teleop extends BaseCodeV3 {



    abstract public void runOpMode() throws InterruptedException;

    public void runOpModeTeleop() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {

            if (odo != null) {
                odo.update();
            }
//             shootingLightIndicator();
            if(gamepad1.a) {
                intake.startIntake();
            } else if (gamepad1.b) {
                intake.stopIntake();
            } else if (gamepad1.x && shooter.checkShootPoosible()) {
                shooter.setupFlywheels();
                shooter.setupAngler();
                sleep(10);
                shooter.shoot();
                telemetry.addLine("shooting");
            } else if (gamepad1.y) {
                shooter.stop();
            }
            drive.setDrivePowers(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
//            telemetry.addData("Green: ", color.green());
//            telemetry.addData("Blue: ", color.blue());
//            telemetry.addData("Red: ", color.red());
//            shooterLeft.setPower(0.5);
//            shooterRight.setPower(0.5);
            if (sensors.getColorRed() > 1500) {

                for (int i = 0; i < 1; i++) {


                    sensors.setLight(sensors.LIGHTPURPLE);
                    sleep(500);
                    sensors.setLight(0);

                }

            }

            if (sensors.getColorGreen()>2000) {

                for (int i = 0; i < 1; i++) {

                    sensors.setLight(sensors.LIGHTGREEN);
                    sleep(500);
                    sensors.setLight(0);


                }
            }
//            telemetry.addData("Motor Current in amps (left): ", shooterLeft.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Motor Current in amps (right): ", shooterRight.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Target ticks;", anglerLeft.getTargetPosition());
//            telemetry.addData("Current ticks;", anglerLeft.getCurrentPosition());
            telemetry.update();
        }
    }

}
