package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
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
            double distance = shootingLightIndicator();
            if(gamepad1.a) {
                intake.startIntake();
            } else if (gamepad1.b) {
                intake.stopIntake();
            } else if (gamepad1.x && shooting_okay()) {
                shooter.setupShoot();
                shooter.shoot();
            } else if (gamepad1.y) {
                shooter.stop();
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
