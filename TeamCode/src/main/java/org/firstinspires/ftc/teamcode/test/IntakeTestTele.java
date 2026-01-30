//package org.firstinspires.ftc.teamcode.test;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.FTC26502OpMode;
//
//@TeleOp
//public class IntakeTestTele extends FTC26502OpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//        waitForStart();
//        super.initOpMode(false, false, false, true, false, false, true);
//        ;
//
//        while(opModeIsActive()) {
//            if (gamepad1.a) {
//                intake.startIntake();
//            }
//            if (gamepad1.b) {
//                intake.firstRowStart();
//            }
//            if (gamepad1.x) {
//                intake.secondRowStart();
//            }
//            if (gamepad1.y) {
//                intake.thirdRowStart();
//            }
//            if (gamepad2.a) {
//                intake.stopIntake();
//            }
//            if (gamepad2.b) {
//                intake.firstRowStop();
//            }
//            if (gamepad2.x) {
//                intake.secondRowStop();
//            }
//            if (gamepad2.y) {
//                intake.thirdRowStop();
//            }
//        }
//    }
//}
