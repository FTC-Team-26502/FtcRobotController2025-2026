package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class BaseCodeV3 extends LinearOpMode {

    public MecanumDrive drive;
    public ShooterSystem shooter;
    public IntakeSystem intake;
    public SensorSystem sensors;


    public void initRobot(boolean useDrive, boolean useOdo, boolean useShooter, boolean useIntake, boolean useSensors) {

        if(useDrive) {Pose2d startPose = new Pose2d(-56, 56, Math.toRadians(-35)); drive = new MecanumDrive(hardwareMap, startPose);}
        if (useShooter) shooter = new ShooterSystem(hardwareMap, telemetry);
        if (useIntake) intake = new IntakeSystem(hardwareMap, telemetry);
        if (useSensors) sensors = new SensorSystem(hardwareMap, telemetry);

        telemetry.addLine("Robot Init Complete");
        telemetry.update();
    }
}
