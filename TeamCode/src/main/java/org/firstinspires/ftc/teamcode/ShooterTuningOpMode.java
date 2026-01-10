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
    public static double shooterPowerLeft = 1200;
    public static double shooterPowerRight = 1200;
    public static double intakePower = 1;
    public static double tickPerRev = 1400.0/ 360;

    public static double ANGLE_DEADBAND_DEG = 3.0;
    private boolean angleLocked = false;
    private double lastAngleDeg = angleDeg;


    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        telemetry.addLine("Init complete. Adjust variables in Dashboard > Config.");
        telemetry.update();

        super.initOpMode(false, false, true, true, false, true, true);

        waitForStart();
        long startMs = System.currentTimeMillis();

        while (opModeIsActive()) {
            double time = (System.currentTimeMillis() - startMs) / 1000.0;

            int targetTicks = (int)(angleDeg * tickPerRev);

            int leftErrorTicks =
                    targetTicks - shooter.anglerLeft.getCurrentPosition();

            double leftErrorDeg = leftErrorTicks / tickPerRev;

            // check if angle changed and set the

            AprilTagDetection tag = vision.checkTag();
            telemetry.addData("Tag (intialized in tuning op mode)", tag);


            if (tag != null) {
                telemetry.addData("dist", shooter.getDistanceToTargetMeters(tag));
                telemetry.addData("Heading", tag.rawPose.x);
            }
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
            if (angleDeg != lastAngleDeg) {
                angleLocked = false;
                lastAngleDeg = angleDeg;
            }

            if (!angleLocked) {

                shooter.anglerLeft.setTargetPosition(targetTicks);
                shooter.anglerRight.setTargetPosition(targetTicks);

                shooter.anglerLeft.setPower(0.2);
                shooter.anglerRight.setPower(0.2);

                if (Math.abs(leftErrorDeg) <= ANGLE_DEADBAND_DEG) {
                    angleLocked = true;

                    shooter.anglerLeft.setPower(0.04);
                    shooter.anglerRight.setPower(0.04);
                }
            }
            telemetry.addData("angleLeft", (shooter.anglerLeft.getCurrentPosition()/(int)(tickPerRev)));
            telemetry.addData("angleRight", (shooter.anglerRight.getCurrentPosition()/(int)(tickPerRev)));
            telemetry.addData("Difference between target and current (left)", (shooter.anglerLeft.getCurrentPosition() - shooter.anglerLeft.getTargetPosition()));
            telemetry.addData("Difference between target and current (right)", (shooter.anglerRight.getCurrentPosition() - shooter.anglerRight.getTargetPosition()));
            telemetry.update();
            intake.br.setPower(intakePower);
            intake.bl.setPower(intakePower);
            intake.fr.setPower(-intakePower);
            intake.mr.setPower(intakePower);
            intake.ml.setPower(intakePower);
            // Optional: richer Dashboard packet (graphs, overlays)
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("time", time);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            sleep(20);
        }
    }
}