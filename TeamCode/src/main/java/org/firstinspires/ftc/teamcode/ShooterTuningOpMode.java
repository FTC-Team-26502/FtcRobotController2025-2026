package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
@TeleOp
public class ShooterTuningOpMode extends FTC26502OpMode {

    public static double angleDeg = 45;

    public static double shooterPower = 0.2;

    public double relativeHeading = 0;

    public static double distanceToTag = 0;

    public static double leftVelocity = 0.0;
    public static double rightVelocity = 0.0;

    public static double shooterPowerLeft = 0.2;
    public static double shooterPowerRight = 0.2;
    public static double intakePower = 1;
    public static double P = 0.032;
    public static double I = 0.000;
    public static double D = 0.0022;
    public static double FLeft = 11.7;
    public static double FRight = 11.7;


    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        telemetry.addLine("Init complete. Adjust variables in Dashboard > Config.");
        telemetry.update();

        initOpMode(false, false, true, false, false, false, false);

        waitForStart();
        long startMs = System.currentTimeMillis();
        while (opModeIsActive()) {
            double time = (System.currentTimeMillis() - startMs) / 1000.0;
            PIDFCoefficients pidfLeft = new PIDFCoefficients(P, I, D, FRight);
            PIDFCoefficients pidfRight = new PIDFCoefficients(P, I, D, FLeft);
            shooter.shooterLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfLeft);
            shooter.shooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfRight);
            shooter.bottomShot(1000,56.0);
            telemetry.update();


            // Optional: richer Dashboard packet (graphs, overlays)
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("time", time);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            sleep(20);
        }
    }
}
