package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public abstract class BaseCodeV3 extends LinearOpMode {

    public MecanumDrive drive;
    public ShooterSystem shooter;
    public IntakeSystem intake;
    public SensorSystem sensors;

    public void initRobot(boolean useDrive, boolean useOdo, boolean useShooter, boolean useIntake, boolean useSensors) {

        if(useDrive) {Pose2d startPose = new Pose2d(-56, 56, Math.toRadians(-35)); drive = new MecanumDrive(hardwareMap, startPose);}
        if (useShooter) shooter = new ShooterSystem(hardwareMap);
        if (useIntake) intake = new IntakeSystem(hardwareMap);
        if (useSensors) sensors = new SensorSystem(hardwareMap);

        telemetry.addLine("Robot Init Complete");
        telemetry.update();
    }
}
//
//class ShooterSystem {
//
//    private DcMotorEx anglerLeft, anglerRight;
//    private DcMotorEx shooterLeft, shooterRight;
//    private DistanceSensor blDist, brDist;
//
//    private final double ANGLER_SPEED = 0.05;
//    private final double ANGLE_TO_TICKS = (1/360.0) * 537.6;
//    private final double DELTA_Y = 0.9;
//    private final double GRAVITY = 9.80665;
//    protected final int SPEED_MULTIPLIER = 45;
//    protected final int ANGLE_OF_SHOOTER = 45;
//
//    ShooterSystem(HardwareMap hw) {
//        anglerLeft = hw.get(DcMotorEx.class, "anglerLeft");
//        anglerRight = hw.get(DcMotorEx.class, "anglerRight");
//        shooterLeft = hw.get(DcMotorEx.class, "shooterLeft");
//        shooterRight = hw.get(DcMotorEx.class, "shooterRight");
//
//        blDist = hw.get(DistanceSensor.class, "blDistance");
//        brDist = hw.get(DistanceSensor.class, "brDistance");
//
//        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        anglerLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        anglerRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        anglerLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        anglerRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//
//        anglerRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        anglerLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//    }
//
//    double calcSpeed() {
//        double d = (blDist.getDistance(DistanceUnit.METER) + brDist.getDistance(DistanceUnit.METER)) / 2;
//        return (d * Math.sqrt(GRAVITY / (d - DELTA_Y))) / SPEED_MULTIPLIER;
//    }
//
//    void shoot() {
//        double speed = calcSpeed();
//        spin(speed);
//        angleShooter(ANGLE_OF_SHOOTER);
//    }
//
//    void spin(double speed){
//        shooterLeft.setPower(speed);
//        shooterRight.setPower(speed);
//    }
//
//    void angleShooter(int angle) {
//        int ticks = (int) Math.round(ANGLE_TO_TICKS * ANGLE_OF_SHOOTER);
//        anglerLeft.setPower(ANGLER_SPEED);
//        anglerRight.setPower(ANGLER_SPEED);
//        anglerLeft.setTargetPosition(ticks);
//        anglerRight.setTargetPosition(ticks);
//    }
//
//    void stop() {
//        shooterLeft.setPower(0);
//        shooterRight.setPower(0);
//        anglerLeft.setPower(0);
//        anglerRight.setPower(0);
//    }
//}
//
