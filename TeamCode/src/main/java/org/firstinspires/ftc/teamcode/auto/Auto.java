package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.BaseCodeV3;
import org.firstinspires.ftc.teamcode.MecanumDrive;

/**
 * First version of autonomous code for Decode.
 * Author: dorinamevans@gmail.com
 */
public abstract class Auto extends BaseCodeV3 {
    int ballDistMultipliers;
    int yForBall;
    public void runOpModeAuto() throws InterruptedException {

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
                .splineToConstantHeading(new Vector2d(-5, 5), Math.toRadians(-45));
        Action closeOut2 = tab2.endTrajectory().fresh().build();
        TrajectoryActionBuilder tab3 = tab2.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(0,0), Math.toRadians(-45));


        Action closeOut3 = tab3.endTrajectory().fresh().build();

        ballDistMultipliers = vision.getObeliskPattern();
        telemetry.addData("distmultiplyer", ballDistMultipliers);
        TrajectoryActionBuilder ppg = tab3.endTrajectory().fresh()
                .turnTo(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-65, 12), Math.toRadians(180));
        Action closeOut4ppg = ppg.endTrajectory().fresh().build();
        TrajectoryActionBuilder pgp = tab3.endTrajectory().fresh()
                .turnTo(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-65, -12), Math.toRadians(180));
        Action closeOut4pgp = pgp.endTrajectory().fresh().build();
        TrajectoryActionBuilder gpp = tab3.endTrajectory().fresh()
                .turnTo(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-65, -36), Math.toRadians(180));
        Action closeOut4gpp = gpp.endTrajectory().fresh().build();
        TrajectoryActionBuilder tab5gpp = gpp.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(-20, 20), Math.toRadians(180))
                .turnTo(Math.toRadians(-40));
        Action closeOut5gpp = tab5gpp.endTrajectory().fresh().build();
        TrajectoryActionBuilder tab5pgp = pgp.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(-20, 20), Math.toRadians(180))
                .turnTo(Math.toRadians(-40));
        Action closeOut5pgp = tab5pgp.endTrajectory().fresh().build();
        TrajectoryActionBuilder tab5ppg = ppg.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(-20, 20), Math.toRadians(180))
                .turnTo(Math.toRadians(-40));
        Action closeOut5ppg = tab5pgp.endTrajectory().fresh().build();
        // Build the trajectory actions
        Action traj1 = tab1.build();
        Action traj2 = tab2.build();
        Action traj3 = tab3.build();
        Action traj4gpp = gpp.build();
        Action traj4ppg = ppg.build();
        Action traj4pgp = pgp.build();
        Action traj5gpp = tab5gpp.build();
        Action traj5ppg = tab5ppg.build();
        Action traj5pgp = tab5pgp.build();

        // Wait for the start command
        waitForStart();
        if (isStopRequested()) return;

        // Run everything sequentially
        Actions.runBlocking(
                new SequentialAction(
                        traj1,
                        closeOut1,

                        new Action() {
                            @Override
                            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                telemetry.addLine("Before shoot");
                                telemetry.update();
                                return false;
                            }
                        },
                        // Example shooter action; ensure shoot() returns Action
                        shooter.setupShootAction(),
                        new Action() {
                            @Override
                            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                telemetry.addLine("Done shoot");
                                telemetry.update();
                                return false;
                            }
                        },
                        // Sleep actions; ensure sleepAction returns Action
                        new SleepAction(1),
                        shooter.shootAction(),
                        new SleepAction(1),
                        traj2,
                        closeOut2,

                        // Intake actions; ensure these return Action
                        intake.startIntakeAction(),
                        new Action() {
                            @Override
                            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                telemetry.update();
                                return false;
                            }
                        },
                        new SleepAction(0.500),
                        intake.stopIntakeAction(),
                        new SleepAction(0.500),
                        traj3,
                        closeOut3,
                        new SleepAction(1.000),
                        intake.startIntakeAction(),
                        new SleepAction(4.000),

                        shooter.stopAction(),
//
                        traj4,
                        closeOut4,
                        new SleepAction(0.500),
                        traj5,
                        closeOut5,
                        // Example shooter action; ensure shoot() returns Action
                        shooter.setupShootAction(),
                        new Action() {
                            @Override
                            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                telemetry.addLine("Done shoot");
                                telemetry.update();
                                return false;
                            }
                        },
                        // Sleep actions; ensure sl         eepAction returns Action
                        new SleepAction(1),
                        shooter.shootAction(),
                        new SleepAction(1),
                        shooter.stopAction()
//
//
//                        // Final shot
////                        shooter.setupShoot(),
////                        new SleepAction(500),
////                        shooter.stop()
                )
        );
    }
}