package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class NewTest extends BaseCodeV2{


    private CRServo inFrontLeft;
    private CRServo inFrontRight;
    private CRServo inMiddleLeft;
    private CRServo inMiddleRight;
    private CRServo inBackLeft;
    private CRServo inBackRight;

    protected final double LIGHTRED = 0.3;
    protected final double LIGHTYELLOW = 0.388;
    protected final double LIGHTGREEN = 0.477;
    protected final double LIGHTBLUE = 0.611;
    protected final double LIGHTPURPLE = 0.722;
    double Colors = LIGHTGREEN;
    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode(true,true,true,true,true);
        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.a) {
                intakeRun(1);
            } else if (gamepad1.b) {
                intakeRun(0);
            } else if (gamepad1.x) {
                shoot(40, 1);
            } else if (gamepad1.y) {
                anglerLeft.setTargetPosition(0);
                anglerRight.setTargetPosition(0);
                stop_shooter();
            }
            driveSpeed(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            telemetry.addData("Green: ", color.green());
            telemetry.addData("Blue: ", color.blue());
            telemetry.addData("Red: ", color.red());
            light.setPosition(Colors);
            if (color.green() > 1000) {
                telemetry.addLine("Ball collected");
                Colors = LIGHTPURPLE;
                sleep(1000);


            }

            if (color.blue() > 1000) {
                telemetry.addLine("Ball collected");
                Colors = LIGHTPURPLE;
                sleep(1000);




            }

            if (color.red() > 1000) {
                telemetry.addLine("Ball collected");
                Colors = LIGHTPURPLE;
                sleep(1000);




            }

            telemetry.update();
        }
    }
}
