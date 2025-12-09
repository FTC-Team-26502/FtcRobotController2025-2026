package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

class IntakeSystem {

    private CRServo fl, fr, ml, mr, bl, br;

    IntakeSystem(HardwareMap hw) {
        fl = hw.get(CRServo.class, "inFrontLeft");
        fr = hw.get(CRServo.class, "inFrontRight");
        ml = hw.get(CRServo.class, "inMiddleLeft");
        mr = hw.get(CRServo.class, "inMiddleRight");

        fl.setDirection(CRServo.Direction.REVERSE);
        ml.setDirection(CRServo.Direction.REVERSE);
    }


    public Action stopIntake(){
        return buildStop();
    }
    public Action buildStop(){
        return new SequentialAction(
                new stop()
        );
    }
    public class stop implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            fl.setPower(0);
            fr.setPower(0);
            ml.setPower(0);
            mr.setPower(0);
            return true;
        }

    }
    public Action startIntake(){
        return buildStart();
    }
    public Action buildStart(){
        return new SequentialAction(
                new Start()
        );
    }
    public class Start implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            packet.addLine("Intake");
            fl.setPower(1);
            fr.setPower(1);
            ml.setPower(1);
            mr.setPower(1);
            packet.addLine("Intake");
            return false;
        }

    }
    public class runIntake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            fl.setPower(1);
            fr.setPower(1);
            ml.setPower(1);
            mr.setPower(1);
            return false;
        }

    }

    public class stopIntake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            fl.setPower(0);
            fr.setPower(0);
            ml.setPower(0);
            mr.setPower(0);
            return false;
        }

    }
}