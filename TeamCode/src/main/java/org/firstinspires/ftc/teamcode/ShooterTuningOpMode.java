package org.firstinspires.ftc.teamcode;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Shooter PID Tuning")
public class ShooterTuningOpMode extends FTC26502OpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        initOpMode(false, false, true, false, false, false, false);

        telemetry.addLine("Shooter PID Tuning Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            shooter.setupFlywheelsPIDTuning();

            telemetry.update();
            sleep(20);
        }
    }
}
