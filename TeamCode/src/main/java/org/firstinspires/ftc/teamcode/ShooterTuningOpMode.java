package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp
public class ShooterTuningOpMode extends FTC26502OpMode {

    public static double angleDeg = 45;

    public static double shooterPower = 0;

    public double relativeHeading = 0;

    public static double distanceToTag = 0;

    public static double leftVelocity = 0.0;
    public static double rightVelocity = 0.0;

    public static double shooterPowerLeft = 0;
    public static double shooterPowerRight = 0;
    public static double intakePower = 1;

    public boolean toggleB = false;

    public boolean toggleX = false;




    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        telemetry.addLine("Init complete. Adjust variables in Dashboard > Config.");
        telemetry.update();

        initOpMode(true, false, true, true, false, true, true);

        waitForStart();
        long startMs = System.currentTimeMillis();

        while (opModeIsActive()) {
            double time = (System.currentTimeMillis() - startMs) / 1000.0;


            // check if angle changed and set the
            if (vision.checkTag() != null) {
                relativeHeading = vision.getHeading();
                distanceToTag = vision.checkTag().ftcPose.z;
                telemetry.addData("angle", angleDeg);
                telemetry.addData("shooter power", shooterPower);
                telemetry.addData("left velocity", shooter.shooterLeft.getVelocity());
                telemetry.addData("right velocity", shooter.shooterRight.getVelocity());
                telemetry.addData("Tag distance", distanceToTag);
                telemetry.addData("Facing: ", relativeHeading);
//                shooter.shooterLeft.setPower(shooterPower);
//                shooter.shooterRight.setPower(shooterPower);
                if (gamepad1.a) {

                    drive.turnDegreesEncoder(-relativeHeading,0.3);
                    telemetry.addLine("turning");
                }

            }


            if (gamepad1.b) {

                toggleB = !toggleB;

                if (toggleB) {
//
//                    intake.fr.setPower(-1);
//                    intake.ml.setPower(1);
//                    intake.mr.setPower(1);
//                    intake.bl.setPower(1);
//                    intake.br.setPower(1);
                    shooterPower = 1100;
                    shooter.shooterLeft.setPower(shooterPower);
                    shooter.shooterRight.setPower(shooterPower);
                    shooter.anglerLeft.setPower(0.2);
                    shooter.anglerRight.setPower(0.2);
                    shooter.anglerLeft.setTargetPosition((int) (45 * (1 / 360.0) * 537.6));
                    shooter.anglerRight.setTargetPosition((int) (45 * (1 / 360.0) * 537.6));
                }
            }

            if (gamepad1.x) {

                toggleX = !toggleX;

                if (toggleX) {
//
                    intake.fr.setPower(-1);
                    intake.ml.setPower(1);
                    intake.mr.setPower(1);
                    intake.bl.setPower(1);
                    intake.br.setPower(1);
                    shooterPower = 1300;
                    shooter.shooterLeft.setPower(shooterPower);
                    shooter.shooterRight.setPower(shooterPower);
                    shooter.anglerLeft.setPower(0.2);
                    shooter.anglerRight.setPower(0.2);
                    shooter.anglerLeft.setTargetPosition((int) (46 * (1 / 360.0) * 537.6));
                    shooter.anglerRight.setTargetPosition((int) (46 * (1 / 360.0) * 537.6));
                }
            }

            telemetry.addData("angle", angleDeg);
            telemetry.addData("shooter power", shooterPower);
//            telemetry.addData("shooter power right", shooterPowerRight);
            telemetry.addData("intake power", intakePower);
            telemetry.addData("left velocity", shooter.shooterLeft.getVelocity());
            telemetry.addData("right velocity", shooter.shooterRight.getVelocity());
            telemetry.addData("power", shooter.shooterLeft.getPower());
//            shooter.shooterLeft.setPower(shooterPowerLeft);
//            shooter.shooterRight.setPower(shooterPowerRight);
//            shooter.anglerLeft.setPower(0.2);
//            shooter.anglerRight.setPower(0.2);
//            shooter.anglerLeft.setTargetPosition((int)(angleDeg*(1/360.0) * 537.6));
//            shooter.anglerRight.setTargetPosition((int)(angleDeg*(1/360.0) * 537.6));
            telemetry.update();


            // Optional: richer Dashboard packet (graphs, overlays)
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("time", time);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            sleep(20);
        }
    }
}
