package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSystem {

    public final CRServo fr, ml, mr, bl, br;
    private final Telemetry telemetry;

    IntakeSystem(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;

        fr = hw.get(CRServo.class, "inFrontRight");
        ml = hw.get(CRServo.class, "inMiddleLeft");
        mr = hw.get(CRServo.class, "inMiddleRight");
        bl = hw.get(CRServo.class, "inBackLeft");
        br = hw.get(CRServo.class, "inBackRight");

        ml.setDirection(CRServo.Direction.REVERSE);
        bl.setDirection(CRServo.Direction.REVERSE);

        telemetry.addLine("Intake initialized");
    }

    /* =====================
       Core Intake Methods
       ===================== */

    public void startIntake() {
        startFirstRow();
        startSecondRow();
        startBackRow();
    }

    public void stopIntake() {
        stopFirstRow();
        stopSecondRow();
        stopBackRow();
    }

    /* =====================
       Row Control Methods
       ===================== */

    public void startFirstRow() {
        fr.setPower(-1);
    }

    public void stopFirstRow() {
        fr.setPower(0);
    }

    public void startSecondRow() {
        ml.setPower(1);
        mr.setPower(1);
    }

    public void stopSecondRow() {
        ml.setPower(0);
        mr.setPower(0);
    }

    public void startBackRow() {
        bl.setPower(1);
        br.setPower(1);
    }

    public void stopBackRow() {
        bl.setPower(0);
        br.setPower(0);
    }

    /* =====================
       Action Wrappers
       (Required Structure)
       ===================== */

    public Action startIntakeAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                startIntake();
                return false;
            }
        };
    }

    public Action stopIntakeAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                stopIntake();
                return false;
            }
        };
    }

    public Action startFirstRowAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                startFirstRow();
                return false;
            }
        };
    }

    public Action stopFirstRowAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                stopFirstRow();
                return false;
            }
        };
    }

    public Action startSecondRowAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                startSecondRow();
                return false;
            }
        };
    }

    public Action stopSecondRowAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                stopSecondRow();
                return false;
            }
        };
    }

    public Action startBackRowAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                startBackRow();
                return false;
            }
        };
    }

    public Action stopBackRowAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                stopBackRow();
                return false;
            }
        };
    }
}