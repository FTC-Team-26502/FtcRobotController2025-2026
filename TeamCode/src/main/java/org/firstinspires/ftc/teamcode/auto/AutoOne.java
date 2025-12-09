package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
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

/**
 * First version of autonomous code for Decode.
 * @author dorinamevans@gmail.com
 */
@Autonomous
public class AutoOne extends BaseCodeV2{
    private MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode(false, true, true, true, true);
        Pose2d startPose = new Pose2d(-56, 56, Math.toRadians(-35));
        drive = new MecanumDrive(hardwareMap, startPose);
        // Build your trajectory sequence
        TrajectoryActionBuilder traj = drive.actionBuilder(startPose)
                .splineToConstantHeading(new Vector2d (20,-20), Math.toRadians(-45));

        Action trajectoryActionCloseOut = traj.endTrajectory().fresh()
                        .build();
        waitForStart();


        Action trajectory = traj.build();
        // Follow the trajectory sequence synchronously (blocks until finished)
        Actions.runBlocking(
            new SequentialAction(
                    trajectory,
                    trajectoryActionCloseOut
            )
        );
        shoot();
        sleep(1000);
        intakeRun(1);
        sleep(500);
        intakeRun(0);
        sleep(1000);
        intakeRun(1);
        sleep(5000);
        intakeRun(0);
    }
}
