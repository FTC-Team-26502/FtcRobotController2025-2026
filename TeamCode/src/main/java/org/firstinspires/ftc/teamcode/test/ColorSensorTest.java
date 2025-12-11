//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import java.util.Locale;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
//@TeleOp
//public class ColorSensorTest extends BaseCodeV3{
//    @Override
//    public void runOpMode() throws InterruptedException {
//        initOpMode(true, true, false, false, true, true);
//        waitForStart();
//        while (opModeIsActive()) {
//            } if(color.green()>2000){
//                telemetry.addLine("Green Ball Detected");
//                sleep(5000);
//            } else if (color.red()>1500) {
//                telemetry.addLine("Purple Ball Detected");
//            }
//            telemetry.addData("Green: ", color.green());
//            telemetry.addData("Blue: ", color.blue());
//            telemetry.addData("Red: ", color.red());
//            telemetry.update();
//        }
//
//    }
//
