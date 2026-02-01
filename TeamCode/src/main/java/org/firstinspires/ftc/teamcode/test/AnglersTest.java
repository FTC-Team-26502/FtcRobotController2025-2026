package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.FTC26502OpMode;
import org.firstinspires.ftc.teamcode.teleop.TeleopWithActions;
@Config
@TeleOp
public class AnglersTest extends FTC26502OpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        shooterIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

}