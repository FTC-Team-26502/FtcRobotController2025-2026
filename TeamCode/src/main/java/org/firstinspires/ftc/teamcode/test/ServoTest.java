package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Testing", group = "Teleop")
public class ServoTest extends LinearOpMode {
    public Servo servo1;
    public Servo servo2;

    protected void initOpMode() {

        servo1 = hardwareMap.get(Servo.class, "Servo 1");

        servo2 = hardwareMap.get(Servo.class, "Servo 2");

        servo1.setDirection(Servo.Direction.FORWARD);
        servo2.setDirection(Servo.Direction.REVERSE);


    }


    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode();

        waitForStart();
        while (opModeIsActive()) {

            servo1.setPosition(0.0);
            servo2.setPosition(1.0);

        }
    }
}
