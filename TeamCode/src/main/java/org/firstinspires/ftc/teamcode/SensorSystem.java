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

    protected SensorSystem(HardwareMap hw, Telemetry telemetry) {
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

    public boolean isPurpleBall() {
        return color.red() > 1500;
    }

    public boolean isGreenBall() {
        return color.green() > 2000;
    }

    public void updateIndicatorLights(double currentTime) {
        light.getPosition();
        if ( isPurpleBall() ) {
            setLight(LIGHTPURPLE);
            timer = currentTime;
        } else if (isGreenBall() ) {
            setLight(LIGHTGREEN);
            timer = currentTime;
        }
        if (currentTime - timer > 5) {
            if (canShoot) {
                setLight(LIGHTBLUE);
            } else {
                setLight(LIGHTRED);
            }
        }
    }

    public void setCanShoot(boolean canShoot) {
        this.canShoot = canShoot;
    }
}
