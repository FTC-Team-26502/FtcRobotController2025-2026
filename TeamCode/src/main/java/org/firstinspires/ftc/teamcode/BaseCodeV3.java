package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public abstract class BaseCodeV3 extends LinearOpMode {

    protected MecanumDrive drive;
    protected ShooterSystem shooter;
    protected IntakeSystem intake;
    protected SensorSystem sensors;
    protected VisionSystem vision;
    protected GoBildaPinpointDriver odo;
    protected boolean blueAlliance;


    public void initOpMode(boolean useDrive, boolean useOdo,
                          boolean useShooter, boolean useIntake,
                          boolean useSensors, boolean useVision,
                           boolean blueAlliance) {
        if (useDrive) {
            Pose2d startPose = new Pose2d(-56, 56, Math.toRadians(-35));
            drive = new MecanumDrive(hardwareMap, startPose);
        }
        if (useShooter) shooter = new ShooterSystem(hardwareMap, telemetry);
        if (useIntake) intake = new IntakeSystem(hardwareMap, telemetry);
        if (useSensors) {
            sensors = new SensorSystem(hardwareMap, telemetry);
            sensors.setLight(SensorSystem.LIGHTRED);
        }
        if (useVision) vision = new VisionSystem(hardwareMap, telemetry, blueAlliance);
        //Init odometry
        if(useOdo) {
            odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            odo.setOffsets(-82.0, -10.0, DistanceUnit.MM);
            odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD);
        }

        this.blueAlliance = blueAlliance;

        telemetry.addLine("Robot Init Complete");
        telemetry.update();
    }

    public boolean shootingLightIndicator() {
        boolean  canShoot = vision.shootingCheck();
        if(!canShoot) {
            sensors.setLight(SensorSystem.LIGHTRED);
        } else {
            sensors.setLight(SensorSystem.LIGHTBLUE);
        }
        return canShoot;
    }
}
