package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.Locale;
import java.util.Map;

@Autonomous
public class AutoOne extends BaseCodeV2{
    private MechanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode(false, true, true, true, true);
        drive = new MechanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-56, 56, Math.toRadians(-35));
        drive.setPoseEstimate(startPose);
        // Build your trajectory sequence
        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .forward(-30)                           // go forward 24 in
                .build();
        waitForStart();
        
        
        // Follow the trajectory sequence synchronously (blocks until finished)
        drive.followTrajectorySequence(traj);
        telemetry.addLine("Stopped shooting now");
        shoot();
    }
}
