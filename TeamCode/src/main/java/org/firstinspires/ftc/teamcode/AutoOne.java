package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.StructSchema;
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
public class AutoOne extends BaseCodeV3{

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot(true, true, true, true, true);
        Pose2d startPose = new Pose2d(-56, 56, Math.toRadians(-35));
        drive = new MecanumDrive(hardwareMap, startPose);
        // Build your trajectory sequence
        TrajectoryActionBuilder traj1 = drive.actionBuilder(startPose)
                .splineToConstantHeading(new Vector2d (20,-20), Math.toRadians(-45));

        Action trajectoryActionCloseOut1 = traj1.endTrajectory().fresh()
                        .build();
        TrajectoryActionBuilder traj2 = drive.actionBuilder(startPose)
                .splineTo(new Vector2d(-12,30),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-30, 30), Math.toRadians(0))
                .splineTo(new Vector2d (20,-20), Math.toRadians(-45));

        Action trajectoryActionCloseOut2 = traj2.endTrajectory().fresh()
                .build();
        waitForStart();


        Action trajectory1 = traj1.build();
        Action trajectory2 = traj2.build();
        // Follow the trajectory sequence synchronously (blocks until finished)
        Actions.runBlocking(
            new SequentialAction(
                    trajectory1,
                    trajectoryActionCloseOut1,
                    shooter.shoot(),
                    sleep(500),
                    intake.run(),
                    sleep(500),
                    intake.stop(),
                    sleep(1000),
                    intake.run(),
                    sleep(3000),
                    trajectory2,
                    trajectoryActionCloseOut2,
                    shooter.shoot()
            )
        );
    }
}
