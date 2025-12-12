package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class StepFromAction implements Step {
    private final Action inner;
    public StepFromAction(Action inner) { this.inner = inner; }
    @Override public Status runStep(TelemetryPacket packet) {
        boolean keep = inner.run(packet);
        return keep ? Status.RUNNING : Status.SUCCESS;
    }
}
