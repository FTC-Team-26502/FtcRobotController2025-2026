package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoTop", group = "Autonomous")
public class AutoTop extends Auto2{

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize robot hardware and subsystems
        initOpMode(true, true, true, true, true,
                true, true);
        super.runOpModeAuto2();
    }

}
