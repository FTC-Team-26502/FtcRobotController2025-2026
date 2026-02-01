package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.FTC26502OpMode;


@TeleOp(name = "Motor test")
@Config
public class MotorTest extends FTC26502OpMode {
    public static boolean shooterLeft;
    public static boolean shooterRight;
    public static boolean viperLeft;
    public static boolean viperRight;
    public static boolean backLeft;
    public static boolean backRight;
    public static boolean frontLeft;
    public static boolean frontRight;
    private DcMotorEx shooterLeftMotor, shooterRightMotor, viperLeftMotor, viperRightMotor, backLeftMotor, backRightMotor, frontLeftMotor, frontRightMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        if(shooterLeft) {
            shooterLeftMotor = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        }
        if(shooterRight) {
            shooterRightMotor = hardwareMap.get(DcMotorEx.class, "shooterRight");
        }
        if(viperLeft) {
            viperLeftMotor = hardwareMap.get(DcMotorEx.class, "viperLeft");
        }
        if(viperRight) {
            viperRightMotor = hardwareMap.get(DcMotorEx.class, "viperRight");
        }
        if(backLeft) {
            backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeft");
            backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if(backRight) {
            backRightMotor = hardwareMap.get(DcMotorEx.class, "backRight");
            backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if(frontLeft) {
            frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeft");
            frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if(frontRight) {
            frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
            frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        waitForStart();

        while(opModeIsActive()) {
            if(shooterLeft){
                shooterLeftMotor.setPower(0.2);
            }
            if(shooterRight){
                shooterRightMotor.setPower(0.2);
            }
            if(viperLeft){
                viperLeftMotor.setPower(0.2);
            }
            if(viperRight){
                viperRightMotor.setPower(0.2);
            }
            if(backLeft){
                backLeftMotor.setPower(0.2);
            }
            if(backRight){
                backRightMotor.setPower(0.2);
            }
            if(frontLeft){
                frontLeftMotor.setPower(0.2);
            }
            if(frontRight){
                frontRightMotor.setPower(0.2);
            }

        }

    }
}
