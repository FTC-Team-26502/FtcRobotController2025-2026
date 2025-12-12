package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class SensorSystem {

    protected final Servo light;
    protected final ColorSensor color;
    protected final DistanceSensor fl;
    protected final DistanceSensor fr;

    public static final double LIGHTRED = 0.3;
    public static final double LIGHTYELLOW = 0.388;
    public static final double LIGHTGREEN = 0.477;
    public static final double LIGHTBLUE = 0.611;
    public static final double LIGHTPURPLE = 0.722;

    SensorSystem(HardwareMap hw, Telemetry telemetry) {
        light = hw.get(Servo.class, "light");
        color = hw.get(ColorSensor.class, "color");
        fl = hw.get(DistanceSensor.class, "flDistance");
        fr = hw.get(DistanceSensor.class, "frDistance");
        light.setPosition(0);
    }

    public boolean setLight(double lightValue) {
        light.setPosition(lightValue);
        return false;
    }

    public int getColorRed() {
        return color.red();
    }
    public int getColorGreen() {
        return color.green();
    }


    public Action buildShootingAction() {
        return new SequentialAction();
    }

//    public void setLight(String color) {
//        if (color == "Blue") {
//            light.setPosition(LIGHTBLUE);
//        } else if (color == "Green") {
//            light.setPosition(LIGHTGREEN);
//        } else if (color == "Purple") {
//            light.setPosition(LIGHTPURPLE);
//        } else if (color == "Red") {
//            light.setPosition(LIGHTRED);
//        } else if (color == "Yellow") {
//            light.setPosition(LIGHTYELLOW);
//        }
//    }
//    public class detectBall implements Action {
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet){
//            if (color.green() > 2000) {
//                telemery.addLine("Green Ball Detected");
//                sleep(5000);
//            } else if (color.red() > 1500) {
//                telemetry.addLine("Purple Ball Detected");
//            }
//            telemetry.addData("Green: ", color.green());
//            telemetry.addData("Blue: ", color.blue());
//            telemetry.addData("Red: ", color.red());
//            telemetry.update();
//            return true;
//        }
//
//    }
//    public int detectBall() {
//        if (color.green() > 2000) {
//            telemetry.addLine("Green Ball Detected");
//            sleep(5000);
//        } else if (color.red() > 1500) {
//            telemetry.addLine("Purple Ball Detected");
//        }
//        telemetry.addData("Green: ", color.green());
//        telemetry.addData("Blue: ", color.blue());
//        telemetry.addData("Red: ", color.red());
//        telemetry.update();
//    }
}
