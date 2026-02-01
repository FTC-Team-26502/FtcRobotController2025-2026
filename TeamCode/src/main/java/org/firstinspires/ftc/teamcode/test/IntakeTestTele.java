package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FTC26502OpMode;
import org.firstinspires.ftc.teamcode.teleop.TeleopWithActions;
@Config
@TeleOp
public class IntakeTestTele extends FTC26502OpMode {
    public static boolean bl = false;
    public static boolean br = false;
    public static boolean ml = false;
    public static boolean mr = false;
    public static boolean fr = false;
    @Override
    public void runOpMode() throws InterruptedException {
        super.initOpMode(false, false, false, true, false,false,false);
        waitForStart();
        while (opModeIsActive()){
            if(bl) {
                intake.bl.setPower(1);
                telemetry.addData("bl power", intake.bl.getPower());
                telemetry.addData("bl port", intake.bl.getPortNumber());
                telemetry.update();
            }
            if(br) {
                intake.br.setPower(1);
                telemetry.addData("br power", intake.br.getPower());
                telemetry.addData("br port", intake.br.getPortNumber());
                telemetry.update();
            }
            if(ml) {
                intake.ml.setPower(1);
                telemetry.addData("ml power", intake.ml.getPower());
                telemetry.addData("ml port", intake.ml.getPortNumber());
                telemetry.update();
            }
            if(mr) {
                intake.mr.setPower(1);
                telemetry.addData("mr power", intake.mr.getPower());
                telemetry.addData("mr port", intake.mr.getPortNumber());
                telemetry.update();
            }
            if(fr) {
                intake.fr.setPower(1);
                telemetry.addData("fr power", intake.fr.getPower());
                telemetry.addData("fr port", intake.fr.getPortNumber());
                telemetry.update();
            }
        }
    }

}