package org.firstinspires.ftc.teamcode.messages;

import com.qualcomm.robotcore.hardware.CRServo;

public class IntakeMessage {

    public final double frPower;
    public final double mlPower;
    public final double mrPower;
    public final double blPower;
    public final double brPower;
    public long timestamp;

    public IntakeMessage(CRServo fr, CRServo ml, CRServo mr, CRServo bl, CRServo br) {
        timestamp = System.currentTimeMillis();
        frPower = fr.getPower();
        mlPower = ml.getPower();
        mrPower = mr.getPower();
        blPower = bl.getPower();
        brPower = br.getPower();
    }

}
