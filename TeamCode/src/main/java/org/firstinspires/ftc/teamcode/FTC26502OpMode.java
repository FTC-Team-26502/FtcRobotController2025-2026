package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/***
 * Base class for all FTC26502 OpModes
 * Author: dorinamevans@gmail.com
 * Autor: ...
 */
public abstract class FTC26502OpMode extends LinearOpMode implements Clock{

    protected MecanumDrive drive;
    protected ShooterSystem shooter;
    protected IntakeSystem intake;
    protected SensorSystem sensors;
    protected VisionSystem vision;
    protected GoBildaPinpointDriver odo;
    protected boolean blueAlliance;


    /**
     * 
     * @param useDrive
     * @param useOdo
     * @param useShooter
     * @param useIntake
     * @param useSensors
     * @param useVision
     * @param blueAlliance
     */
    public void initOpMode(boolean useDrive, boolean useOdo,
                          boolean useShooter, boolean useIntake,
                          boolean useSensors, boolean useVision,
                           boolean blueAlliance) {
        if (useDrive) {
            Pose2d startPose = new Pose2d(-56, 56, Math.toRadians(-35));
            drive = new MecanumDrive(hardwareMap, startPose);
        }

        if (useIntake) intake = new IntakeSystem(hardwareMap, telemetry);
        if (useSensors) {
            sensors = new SensorSystem(hardwareMap, telemetry);
            sensors.setLight(SensorSystem.LIGHTRED);
        }
        if (useVision || useShooter) {
            vision = new VisionSystem(hardwareMap, telemetry, blueAlliance);
        }
        if (useShooter ) {
            shooter = new ShooterSystem(hardwareMap, telemetry, vision, false);
        }
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

    public double now() {
        return this.getRuntime();
    }

}
