package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleopBlue extends TeleopWithActions {


    @Override
    public void runOpMode() throws InterruptedException {
        super.initOpMode(true, false, true, true, true, true, true);;
        super.runOpModeTeleop();
    }


}
