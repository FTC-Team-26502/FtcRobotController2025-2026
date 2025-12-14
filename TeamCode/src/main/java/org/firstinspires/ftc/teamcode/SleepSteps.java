package org.firstinspires.ftc.teamcode;

import android.app.Notification;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.SleepAction;

public class SleepSteps implements Step{

    private SleepAction sleeper;
    private double sec;

    public SleepSteps(double sec) {
        this.sleeper = new SleepAction(sec);
        this.sec = sec;
    }

    @Override
    public Status runStep(TelemetryPacket packet) {
        while ( sleeper.run(packet) );
        return Status.SUCCESS;
    }

}
