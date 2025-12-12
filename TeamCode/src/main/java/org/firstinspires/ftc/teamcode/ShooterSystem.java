package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class ShooterSystem {
    private final Telemetry telemetry;
    private DcMotorEx anglerLeft, anglerRight;
    private DcMotorEx shooterLeft, shooterRight;
    private DistanceSensor blDist, brDist;
    private CRServo bl, br;
    private VisionSystem vision;
    protected final IMU shooterIMU;

    private final double ANGLER_SPEED = 0.05;
    private final double ANGLE_TO_TICKS = (1/360.0) * 537.6;
    private final double GRAVITY = 9.80665;
    protected final int SPEED_MULTIPLIER = 10;
    protected final int ANGLE_OF_SHOOTER = 45;

    protected double shootingAngle = Math.PI/4;

    ShooterSystem(HardwareMap hw, Telemetry telemetry, VisionSystem vision) {
        this.vision = vision;
        shooterIMU = hw.get(IMU.class, "shooterIMU");

        // Set how the IMU is mounted on the robot. Update these to match your physical mounting.
        RevHubOrientationOnRobot.LogoFacingDirection logoDir =
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN;      // change as needed
        RevHubOrientationOnRobot.UsbFacingDirection usbDir =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;  // change as needed
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(logoDir, usbDir));
        shooterIMU.initialize(params);
        anglerLeft = hw.get(DcMotorEx.class, "anglerLeft");
        anglerRight = hw.get(DcMotorEx.class, "anglerRight");
        shooterLeft = hw.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hw.get(DcMotorEx.class, "shooterRight");

        blDist = hw.get(DistanceSensor.class, "blDistance");
        brDist = hw.get(DistanceSensor.class, "brDistance");

        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        anglerLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        anglerRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        anglerLeft.setTargetPosition(0);
        anglerRight.setTargetPosition(0);
        anglerLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        anglerRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        anglerRight.setDirection(DcMotorSimple.Direction.REVERSE);
        anglerLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        bl = hw.get(CRServo.class, "inBackLeft");
        br = hw.get(CRServo.class, "inBackRight");

        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        this.telemetry = telemetry;
    }

    // Convert ticks/sec -> m/s
    public double ticksPerSecToMetersPerSec(double ticksPerSec) {
        double ticksPerRev = 28;
        double wheelRadiusMeters = 0.096;
        double revsPerSec = ticksPerSec / ticksPerRev;
        double wheelCircumference = 2.0 * Math.PI * wheelRadiusMeters;
        return revsPerSec * wheelCircumference;
    }

    // Convenience: meters/sec -> ticks/sec (inverse)
    public double metersPerSecToTicksPerSec(double metersPerSec) {
        double ticksPerRev = 28;
        double wheelRadiusMeters = 0.096;
        double wheelCircumference = 2.0 * Math.PI * wheelRadiusMeters;
        double revsPerSec = metersPerSec / wheelCircumference;
        return revsPerSec * ticksPerRev;
    }

    public boolean setupShoot() {
        // Should we check shooting possible here

        double targetDistance = vision.distanceToTarget();;

        double speed = getTargetSpeed(targetDistance); // mps
        if (speed<0) {
            return false;
        }
        double angularSpeed = metersPerSecToTicksPerSec(speed);
        if ( Math.abs(angularSpeed - shooterLeft.getVelocity()) < 0.3 &&
            Math.abs(angularSpeed - shooterRight.getVelocity()) < 0.3 ){
            return true;
        }
        shooterLeft.setVelocity(angularSpeed);
        shooterRight.setVelocity(angularSpeed);
        int ticks = (int) Math.round(ANGLE_TO_TICKS * ANGLE_OF_SHOOTER);
        anglerLeft.setPower(ANGLER_SPEED);
        anglerRight.setPower(ANGLER_SPEED);
        anglerLeft.setTargetPosition(ticks);
        anglerRight.setTargetPosition(ticks);

        telemetry.addLine(String.format("min shooting angle %s",
                shooterIMU.getRobotYawPitchRollAngles()));

        return false;
    }


    public static double selectiveAverage(double value, double a, double b) {
        int count = 1;
        double sum = value;
        if (Math.abs(a - value) <= 0.1) {
            sum += a;
            count++;
        }
        if (Math.abs(b - value) <= 0.1) {
            sum += b;
            count++;
        }
        return sum / count;
    }

    private double getTargetSpeed(double targetDistance) {
        double distance = selectiveAverage(targetDistance,
                blDist.getDistance(DistanceUnit.METER),
                brDist.getDistance(DistanceUnit.METER));
        double targetSpeed = calculateShootingSpeed(distance);
        return targetSpeed;
    }

    public Action setupShootAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                telemetry.addLine("Shooting setup done");
                telemetry.update();
                return setupShoot();
            }
        };
    }

    public void shoot() {
        bl.setPower(1);
        br.setPower(1);
    }

    public Action shootAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                shoot();
                telemetry.addLine("Shooting");
                telemetry.update();
                return false;
            }
        };
    }

    public void stop() {
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        anglerLeft.setPower(0);
        anglerRight.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }

    public Action stopAction(){

        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                stop();
                telemetry.addLine("Shooting stopped");
                telemetry.update();
                return false;
            }
        };
    }

    double calcSpeed() {
        double d = (blDist.getDistance(DistanceUnit.METER) + brDist.getDistance(DistanceUnit.METER)) / 2;
        return (d * Math.sqrt(GRAVITY / (d - VisionSystem.DELTA_Y))) / SPEED_MULTIPLIER;
    }

    public double calculateShootingSpeed(double d){
        //
        double deltaH = VisionSystem.DELTA_Y;
        if (d * Math.tan(shootingAngle) - deltaH > 0) {
            return Math.sqrt((GRAVITY * d * d) /
                    (2.0 * Math.pow(Math.cos(shootingAngle), 2) * (d * Math.tan(shootingAngle) - deltaH)));
        }  else {
            return -1.0;
        }

    }

}

