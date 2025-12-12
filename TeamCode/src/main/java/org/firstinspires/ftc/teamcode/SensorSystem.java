package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.Timer;

public class SensorSystem {

    protected final Servo light;
    protected final ColorSensor color;
    protected final DistanceSensor fl;
    protected final DistanceSensor fr;
    protected boolean canShoot;
    protected double timer;

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




    public void updateIndicatorLights(double currentTime) {
        if (getColorRed() > 1500) {
            setLight(LIGHTPURPLE);
            timer = currentTime;
        } else if (getColorGreen()>2000) {
            setLight(LIGHTGREEN);
            timer = currentTime;
        } else if (currentTime - timer > 5) {
            if (canShoot) {
                setLight(LIGHTBLUE);
            } else {
                setLight(LIGHTRED);
            }
        } else {
            setLight(0);
        }
    }

    public void setCanShoot(boolean canShoot) {
        this.canShoot = canShoot;
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
