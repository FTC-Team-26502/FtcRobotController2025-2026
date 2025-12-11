package org.firstinspires.ftc.teamcode.teleop;

public abstract class TeleopRed extends Teleop {
    @Override
    public void runOpMode() throws InterruptedException {
        super.initOpMode(true, false, true, true, true, true, false);;
        super.runOpModeTeleop();
    }
}
