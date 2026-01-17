package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FTC26502OpMode;
@TeleOp
public class TeleOpSagnikBlue extends TeleopWithActions {
    @Override
    public void runOpMode() throws InterruptedException {
        super.initOpMode(true, false, true, true, true, true, true);;
        super.runOpModeTeleop(true);
    }
}
