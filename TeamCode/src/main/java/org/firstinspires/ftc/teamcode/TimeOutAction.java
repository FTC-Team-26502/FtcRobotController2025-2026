package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class TimeOutAction implements Action {
    private final Action inner;
    private final double timeoutSec;
    private final Clock clock;
    private double start;

    public TimeOutAction(Action inner, double timeoutSec, Clock clock) {
        this.inner = inner;
        this.timeoutSec = timeoutSec;
        this.clock = clock;
        start = -1.0;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        double now = clock.now();
        if (start < 0) {
            start = now;
        }
        boolean keepRunning = inner.run(packet);
        boolean timedOut = now - start >= timeoutSec;
        return keepRunning && !timedOut;
    }

}
