package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSystem {

    protected final CRServo fr, ml, mr, bl, br;
    private final Telemetry telemetry;

    private boolean toggleFirst = false;
    private boolean toggleSecond = false;

    IntakeSystem(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;
        //fl = hw.get(CRServo.class, "inFrontLeft");
        fr = hw.get(CRServo.class, "inFrontRight");
        ml = hw.get(CRServo.class, "inMiddleLeft");
        mr = hw.get(CRServo.class, "inMiddleRight");
        bl = hw.get(CRServo.class, "inBackLeft");
        br = hw.get(CRServo.class, "inBackRight");


        //fl.setDirection(CRServo.Direction.REVERSE);
        ml.setDirection(CRServo.Direction.REVERSE);
        bl.setDirection(CRServo.Direction.REVERSE);
        telemetry.addLine("Intake initialized");
    }

    public Action stopIntakeAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //fl.setPower(0);
                fr.setPower(0);
                ml.setPower(0);
                mr.setPower(0);
                bl.setPower(0);
                br.setPower(0);
                return false;
            }
        };
    }

    public Action startIntakeAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //fl.setPower(1);
                fr.setPower(-1);
                ml.setPower(1);
                mr.setPower(1);
                bl.setPower(1);
                br.setPower(1);
                return false;
            }
        };
    }

    public Action stallDetected() {
        return new Action() {
        @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                fr.setPower(1);
                return false;
            }
        };

    }
    public Action firstRow() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                toggleFirst = !toggleFirst;
                if (toggleFirst) {
                    fr.setPower(-1);
                } else {
                    fr.setPower(0);
                }
                return false;
            }

        };
    }

    public Action secondRow() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                toggleSecond = !toggleSecond;
                if (toggleSecond) {
                    ml.setPower(1);
                    mr.setPower(1);
                } else {
                    ml.setPower(0);
                    mr.setPower(0);
                }
                return false;
            }
        };

    }

}