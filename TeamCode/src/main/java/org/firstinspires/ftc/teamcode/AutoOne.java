package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * First version of autonomous code for Decode.
 * Author: dorinamevans@gmail.com
 */
@Autonomous(name = "AutoOne", group = "Autonomous")
public class AutoOne extends BaseCodeV3 {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize robot hardware and subsystems
        initRobot(true, true, true, true, true);

        // Define start pose (units must match your RR config; inches are common)
        Pose2d startPose = new Pose2d(-56, 56, Math.toRadians(-35));

        // Initialize drive with start pose
        drive = new MecanumDrive(hardwareMap, startPose);

        // Build first trajectory from the start pose
        TrajectoryActionBuilder tab1 = drive.actionBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-12, 12), Math.toRadians(-45));

        // Action that ensures pose is set to end of traj1 (optional)
        Action closeOut1 = tab1.endTrajectory().fresh().build();

        // If you want traj2 to start where traj1 ends, start from endTrajectory()
        TrajectoryActionBuilder tab2 = tab1.endTrajectory().fresh()
                .splineTo(new Vector2d(-12, 30), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-30, 30), Math.toRadians(0))
                .splineTo(new Vector2d(20, -20), Math.toRadians(-45));

        Action closeOut2 = tab2.endTrajectory().fresh().build();

        // Build the trajectory actions
        Action traj1 = tab1.build();
        Action traj2 = tab2.build();

        // Wait for the start command
        waitForStart();
        if (isStopRequested()) return;

        // Run everything sequentially
        Actions.runBlocking(
                new SequentialAction(
                        traj1,
                        closeOut1,

                        // Example shooter action; ensure shoot() returns Action
                        shooter.setupShoot(),

                        // Sleep actions; ensure sleepAction returns Action
                        new SleepAction(0.5),

                        // Intake actions; ensure these return Action
                        intake.startIntake(),
                        new SleepAction(0.500),
                        intake.stopIntake(),
                        new SleepAction(1.000),
                        intake.startIntake(),
                        new SleepAction(3.000),

                        shooter.stop(),

                        traj2,
                        closeOut2,

                        // Final shot
                        shooter.setupShoot(),
                        new SleepAction(500),
                        shooter.shoot()
                )
        );
    }
}