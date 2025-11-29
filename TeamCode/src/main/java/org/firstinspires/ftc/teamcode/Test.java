package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.opencv.core.Mat;

@TeleOp(name="Outtake 1", group="Tests")
public class Test extends LinearOpMode {

    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;

    protected double power = 0.5;
    @Override
    public void runOpMode() {
        leftMotor  = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        
        // Set one motor reversed so they spin opposite directions
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Wait for start
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a){
                power = Math.min(power-0.1, 1.0);
                sleep(250);
            }else if (gamepad1.y){
                power = Math.max(0.0, power+0.1);
                sleep(250);
            }

            // Apply the same power to both motors
//            leftMotor.setPower(power);
//            rightMotor.setPower(power);

            telemetry.addData("Power: ",    power);
            telemetry.update();
        }
    }
}