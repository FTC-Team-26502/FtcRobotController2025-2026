package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.teleop.TeleopBlue;

public class ShooterTuningOpMode extends FTC26502OpMode {

    public static double angleDeg = 45;
    public static double shooterPower = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Init complete. Adjust variables in Dashboard > Config.");
        telemetry.update();

        waitForStart();
        long startMs = System.currentTimeMillis();

        while (opModeIsActive()) {
            double time = (System.currentTimeMillis() - startMs) / 1000.0;
            // check if angle changed and set the

            // Send normal key-value telemetry
            telemetry.addData("gain", gain);
            telemetry.addData("targetTicks", target);
            telemetry.addData("enableThing", enabled);
            telemetry.addData("computed", computed);
            telemetry.update();

            // Optional: richer Dashboard packet (graphs, overlays)
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("computed", computed);
            packet.put("time", time);
            dashboard.sendTelemetryPacket(packet);

            sleep(20);
        }
    }
}
