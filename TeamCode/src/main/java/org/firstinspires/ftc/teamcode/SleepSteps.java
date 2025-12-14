package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.Step;

public class SleepSteps implements Step{

    private double sleeper;

    public SleepSteps(Sleeper sleeper) {
        this.sleeper = sleeper;
    }

    @Override
    public Status runStep(TelemetryPacket packet) {
        return null;
    }

}
