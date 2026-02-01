//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//@TeleOp
//public class AnglerMover extends FTC26502OpMode{
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        initOpMode(false, false, true, false, false, false, false);
//        shooter.anglerLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        shooter.anglerRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        waitForStart();
//        while (opModeIsActive()) {
//            shooter.anglerLeft.setPower(gamepad1.left_stick_y);
//            shooter.anglerRight.setPower(gamepad1.left_stick_y);
//        }
//    }
//}
