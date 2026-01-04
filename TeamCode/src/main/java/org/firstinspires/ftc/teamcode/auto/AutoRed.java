package org.firstinspires.ftc.teamcode.auto;

public class AutoRed extends Auto {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize robot hardware and subsystems
        initOpMode(true, true, true, true, true,
                true, false);
        super.runOpModeAuto();
    }
}
