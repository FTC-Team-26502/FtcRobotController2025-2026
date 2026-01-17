package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FTC26502OpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

/**
 * First version of autonomous code for Decode.
 * Author: dorinamevans@gmail.com
 */
@Config
public abstract class Auto2 extends FTC26502OpMode {
    public static int y = -10;
    public static int x = -10;
    public static int heading = 45;
    public static int angle = 50;
    public static int power = 1350;
    public static int wait = 1;
    public static int y1 = 72;
    public static int x1 = 18;
    public static int x2 = 56;
    public static int heading1 = 45;
    public static int yMultiplier;
    public void runOpModeAuto2() throws InterruptedException {
        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        telemetry.addLine("Init complete. Adjust variables in Dashboard > Config.");
        telemetry.update();
        if(blueAlliance){
            yMultiplier = 1;
        }else{
            yMultiplier = -1;
        }
        // Define start pose (units must match your RR config; inches are common)
        Pose2d startPose = new Pose2d(-56, 56, Math.toRadians(90));

        // Initialize drive with start pose
        drive = new MecanumDrive(hardwareMap, startPose);


        // Build first trajectory from the start pose
        TrajectoryActionBuilder driveToShoot = drive.actionBuilder(startPose)
                .splineTo(new Vector2d(x, y*yMultiplier), Math.toRadians(heading*yMultiplier));
        // Action that ensures pose is set to end of traj1 (optional)
        Action closeDriveToShoot = driveToShoot.endTrajectory().fresh().build();

        TrajectoryActionBuilder thirdRowAndShoot = drive.actionBuilder(new Pose2d(x,y*yMultiplier, Math.toRadians(heading*yMultiplier)))
                .splineTo(new Vector2d(x2,-30*yMultiplier), Math.toRadians(-90*yMultiplier))
                .splineToConstantHeading(new Vector2d(x2,-70*yMultiplier), Math.toRadians(-90*yMultiplier))
                .splineToConstantHeading(new Vector2d(x2,-60*yMultiplier), Math.toRadians(-90*yMultiplier))
                .splineTo(new Vector2d(-12,-12*yMultiplier),Math.toRadians(45*yMultiplier));
        Action closeThridRowAndShoot = thirdRowAndShoot.endTrajectory().fresh().build();

        TrajectoryActionBuilder leave = drive.actionBuilder(new Pose2d(-56,56, Math.toRadians(90))      )
                .splineToConstantHeading(new Vector2d(-42,56),Math.toRadians(90));
        Action closeLeave = leave.endTrajectory().fresh().build();

        // Action that ensures pose is set to end of traj1 (optional)
        Action driveToShootTraj = driveToShoot.build();
        Action thirdRowAndShootTraj = thirdRowAndShoot.build();
        Action leaveTraj = leave.build();

        // Wait for the start command
        waitForStart();
        if (isStopRequested()) return;

        // Run everything sequentially
        Actions.runBlocking(
                new SequentialAction(
                        shooter.shootTop(1500,43,1),
                        new SleepAction(wait),
                        intake.startIntakeAction(),
                        new SleepAction(5),
                        shooter.stopAction(),
                        leaveTraj,
                        closeLeave
//                        thirdRowAndShootTraj,
//                        closeThridRowAndShoot,
//                        intake.stopIntakeAction()
////                        shooter.shootBottom(),
//                        firstRowAndShootTraj,
//                        closeFirstRowAndShoot,
//                        new SleepAction(wait),
////                        shooter.shootBottom(),
//                        new SleepAction(wait),
//                        intake.startIntakeAction(),
//                        new SleepAction(wait),
////                        shooter.shootBottom(),
//                        secondRowAndShootTraj,
//                        closeSecondRowAndShoot,
////                        shooter.shootBottom(),
//                        new SleepAction(wait),
//                        intake.startIntakeAction(),
//                        new SleepAction(wait),
////                        shooter.shootBottom(),
//                        thirdRowAndShootTraj,
//                        closeThridRowAndShoot,
////                        shooter.shootBottom(),
//                        new SleepAction(wait),
//                        intake.startIntakeAction(),
//                        new SleepAction(wait),
////                        shooter.shootBottom(),
//                        leaveTraj,
//                        closeLeave


                )
        );
    }
}