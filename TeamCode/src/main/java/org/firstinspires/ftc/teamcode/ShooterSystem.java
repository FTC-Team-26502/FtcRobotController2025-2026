package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ShooterSystem {
    private final Telemetry telemetry;
    private DcMotorEx anglerLeft, anglerRight;
    private DcMotorEx shooterLeft, shooterRight;
    private DistanceSensor blDist, brDist;
    private CRServo bl, br;

    private final double ANGLER_SPEED = 0.05;
    private final double ANGLE_TO_TICKS = (1/360.0) * 537.6;
    private final double DELTA_Y = 0.9;
    private final double GRAVITY = 9.80665;
    protected final int SPEED_MULTIPLIER = 10;
    protected final int ANGLE_OF_SHOOTER = 45;

    ShooterSystem(HardwareMap hw, Telemetry telemetry) {
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

    public Action setupShoot() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                double speed = calcSpeed();
                shooterLeft.setPower(speed);
                shooterRight.setPower(speed);
                int ticks = (int) Math.round(ANGLE_TO_TICKS * ANGLE_OF_SHOOTER);
                anglerLeft.setPower(ANGLER_SPEED);
                anglerRight.setPower(ANGLER_SPEED);
                anglerLeft.setTargetPosition(ticks);
                anglerRight.setTargetPosition(ticks);
                telemetry.addLine("Shooting setup done");
                telemetry.update();
                return false;
            }
        };

    }

    public Action shoot(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                bl.setPower(1);
                br.setPower(1);
                telemetry.addLine("Shooting");
                telemetry.update();
                return false;
            }
        };
    }

    public Action stop(){

        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                shooterLeft.setPower(0);
                shooterRight.setPower(0);
                anglerLeft.setPower(0);
                anglerRight.setPower(0);
                br.setPower(0);
                bl.setPower(0);
                telemetry.addLine("Shooting stopped");
                telemetry.update();
                return false;
            }
        };
    }

    double calcSpeed() {
        double d = (blDist.getDistance(DistanceUnit.METER) + brDist.getDistance(DistanceUnit.METER)) / 2;
        return (d * Math.sqrt(GRAVITY / (d - DELTA_Y))) / SPEED_MULTIPLIER;
    }

}

