package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

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
    public static double tickPerRev = 1425.2 / 360;


    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        telemetry.addLine("Init complete. Adjust variables in Dashboard > Config.");
        telemetry.update();

        initOpMode(false, false, true, true, false, false, false);

        waitForStart();
        long startMs = System.currentTimeMillis();

        while (opModeIsActive()) {
            double time = (System.currentTimeMillis() - startMs) / 1000.0;


            // check if angle changed and set the
            if (vision.checkTag() != null) {
                relativeHeading = vision.checkTag().ftcPose.yaw;
                distanceToTag = vision.checkTag().ftcPose.z;
                telemetry.addData("angle", angleDeg);
                telemetry.addData("shooter power", shooterPower);
                telemetry.addData("left velocity", shooter.shooterLeft.getVelocity());
                telemetry.addData("right velocity", shooter.shooterRight.getVelocity());
                telemetry.addData("Tag distance", distanceToTag);
                telemetry.addData("Facing: ", relativeHeading);
                shooter.shooterLeft.setPower(shooterPower);
                shooter.shooterRight.setPower(shooterPower);
                telemetry.update();
            }

            AprilTagDetection tag = shooter.vision.checkTag();
            telemetry.addData("angle", angleDeg);
            telemetry.addData("dist", shooter.getDistanceToTargetMeters(tag));
            telemetry.addData("ticksPerRev", tickPerRev);
            telemetry.addData("shooter velocity left", shooterPowerLeft);
            telemetry.addData("shooter velocity right", shooterPowerRight);
            telemetry.addData("intake power", intakePower);
            telemetry.addData("left velocity", shooter.shooterLeft.getVelocity());
            telemetry.addData("right velocity", shooter.shooterRight.getVelocity());
            telemetry.addData(" power % offset", Math.abs(shooter.shooterLeft.getVelocity()-shooter.shooterRight.getVelocity())/shooterPowerLeft);
            telemetry.addData("power", shooter.shooterLeft.getPower());
            shooter.shooterLeft.setVelocity(shooterPowerLeft);
            shooter.shooterRight.setVelocity(shooterPowerRight);
//            shooter.anglerLeft.setPower(0.2);
//            shooter.anglerRight.setPower(0.2);
//            shooter.anglerLeft.setTargetPosition((int)(angleDeg*tickPerRev));
//            shooter.anglerRight.setTargetPosition((int)(angleDeg*tickPerRev));
            telemetry.update();


            // Optional: richer Dashboard packet (graphs, overlays)
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("time", time);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            sleep(20);
        }
    }
}
