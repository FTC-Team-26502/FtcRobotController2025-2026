package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class IntakeSystem {

    private final CRServo fl, fr, ml, mr;
    private final Telemetry telemetry;

    IntakeSystem(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;
        fl = hw.get(CRServo.class, "inFrontLeft");
        fr = hw.get(CRServo.class, "inFrontRight");
        ml = hw.get(CRServo.class, "inMiddleLeft");
        mr = hw.get(CRServo.class, "inMiddleRight");

        fl.setDirection(CRServo.Direction.REVERSE);
        ml.setDirection(CRServo.Direction.REVERSE);
        telemetry.addLine("Intake initialized");
    }


    public Action stopIntake(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                fl.setPower(0);
                fr.setPower(0);
                ml.setPower(0);
                mr.setPower(0);
                return false;
            }
        };
    }

    public Action startIntake(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                telemetry.addLine("Intake Start");
                fl.setPower(1);
                fr.setPower(1);
                ml.setPower(1);
                mr.setPower(1);
                telemetry.addLine("Intake Started");
                telemetry.update();
                return false;
            }
        };
    }

}