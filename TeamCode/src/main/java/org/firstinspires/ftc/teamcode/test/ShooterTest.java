package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(name = "Two Motors Left Stick Y", group = "Linear Opmode")
public class ShooterTest extends LinearOpMode {

    private DcMotorEx left;
    private DcMotorEx right;
    public static double leftVelocity = 2520;
    public static double rightVelocity = 2520;
    public static double P = 7;
    public static double I = 2;
    public static double D = 0;
    public static double FLeft = 12;
    public static double FRight = 12;

    @Override
    public void runOpMode() {

        // Get motors from hardware map
        left  = hardwareMap.get(DcMotorEx.class, "left");
        right = hardwareMap.get(DcMotorEx.class, "right");
        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        telemetry.addLine("Init complete. Adjust variables in Dashboard > Config.");
        telemetry.update();

        // Reverse one motor
        right.setDirection(DcMotor.Direction.REVERSE);

        // Optional but recommended
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            // FTC joystick Y is inverted
            PIDFCoefficients pidfLeft = new PIDFCoefficients(P, I, D, FRight);
            PIDFCoefficients pidfRight = new PIDFCoefficients(P, I, D, FLeft);
            left.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfLeft);
            right.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfRight);


            // Apply same power to both motors
            left.setVelocity(leftVelocity);
            right.setVelocity(rightVelocity);

            telemetry.addData("Left Motor Velocity", left.getVelocity());
            telemetry.addData("Right Motor Velocity", right.getVelocity());
            telemetry.addData("Left Motor Current", left.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Right Motor Current", right.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.update();

        }
    }
}