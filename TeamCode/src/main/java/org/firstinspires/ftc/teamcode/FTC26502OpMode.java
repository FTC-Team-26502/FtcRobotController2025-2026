package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/***
 * Base class for all FTC26502 OpModes
 *
 * <p>Provides initialization for drivetrain, odometry, shooter, intake, sensors, and vision.
 * Subclasses choose which subsystems to enable via {@link #initOpMode(boolean, boolean, boolean, boolean, boolean, boolean, boolean)}.
 * Implements {@link Clock} by exposing {@link #now()}.</p>
 *
 *@author: dorinamevans@gmail.com
 *@author: sagnikbiswas712@gmail.com
 */
public abstract class FTC26502OpMode extends LinearOpMode implements Clock{

    protected static final boolean DEBUG = true;

    protected MecanumDrive drive;
    /** Projectile shooter initialized, nullable if useShooter is false*/
    protected ShooterSystem shooter;
    /** Intake subsystem for taking in artifacts, nullable if useIntake is false*/
    protected IntakeSystem intake;
    protected SensorSystem sensors;

    /** Vision subsystem used for aiming and calculating artifact trajectory, nullable if useIntake is false*/
    protected VisionSystem vision;
    protected GoBildaPinpointDriver odo;
    protected boolean blueAlliance;

    protected FTC26502OpMode() {
        if (DEBUG) {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            telemetry.setMsTransmissionInterval(1000);
            telemetry.setAutoClear(false);
        }
    }


    /**
     * Initializes the selected robot subsystems and sets alliance color.
     *
     * <p>Only the subsystems specified by the boolean flags will be created and initialized.
     * Call this from your OpMode's init phase.</p>
     *
     * @param useDrive     whether to initialize the mecanum drive with a default start pose
     * @param useOdo       whether to initialize GoBilda Pinpoint odometry
     * @param useShooter   whether to initialize the shooter system
     * @param useIntake    whether to initialize the intake system
     * @param useSensors   whether to initialize the sensor system (and set default light color)
     * @param useVision    whether to initialize the vision system (also needed by shooter)
     * @param blueAlliance whether the robot is on the blue alliance (affects vision/shooter behavior)
     */
    public void initOpMode(boolean useDrive, boolean useOdo,
                          boolean useShooter, boolean useIntake,
                          boolean useSensors, boolean useVision,
                           boolean blueAlliance) {
        if (useDrive) {
            Pose2d startPose = new Pose2d(-56, 56, Math.toRadians(-35));
            drive = new MecanumDrive(hardwareMap, startPose);
        }

        if (useIntake) {
            intake = new IntakeSystem(hardwareMap, telemetry);
        }
        if (useSensors) {
            sensors = new SensorSystem(hardwareMap, telemetry);
            sensors.setLight(SensorSystem.LIGHTRED);
        }
        if (useVision || useShooter) {
            /** <p> This is used reading the goal, sending the data {@link VisionSystem#checkTag()}
              and only applies if useShooter is also true. blueAlliance boolean controls what apriltag it reads
             {@link org.firstinspires.ftc.vision.apriltag.AprilTagDetection} </p>*/
            vision = new VisionSystem(hardwareMap, telemetry, blueAlliance);
        }
        if (useShooter ) {
            /** <p> This is for shooting the artifacts and calculating
             *  the trajectory of the artifacts {@link ShooterSystem#calculateShootingSpeed(double, double),
             *  check {@link ShooterSystem#calculateShootingAngle(double)}}.
             *  and setting up the shooter to be ready {@link ShooterSystem#shoot()} and setting up the speed
             *  {@link ShooterSystem#setupFlywheels()} and angle {@link ShooterSystem#setupAngler()} </p>*/
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

    /**
     * Returns the current runtime of this OpMode in seconds.
     *
     * <p>Implements {@link Clock#now()} using {@link #getRuntime()}.</p>
     *
     * @return current runtime in seconds
     */
    public double now() {
        return this.getRuntime();
    }

}
