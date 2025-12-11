package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoBlue", group = "Autonomous")
public class AutoBlue extends Auto{

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize robot hardware and subsystems
        initOpMode(true, true, true, true, true,
                true, true);
        super.runOpModeAuto();
    }

}
