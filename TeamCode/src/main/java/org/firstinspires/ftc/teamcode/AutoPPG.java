package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
public class AutoPPG extends BaseCodeV3 {

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
                .splineToConstantHeading(new Vector2d(-5, 5), Math.toRadians(-45));
        Action closeOut2 = tab2.endTrajectory().fresh().build();
        TrajectoryActionBuilder tab3 = tab2.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(0,0), Math.toRadians(-45));


        Action closeOut3 = tab3.endTrajectory().fresh().build();

        TrajectoryActionBuilder tab4 = tab3.endTrajectory().fresh()
                .turnTo(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-65, 25), Math.toRadians(180));
        Action closeOut4 = tab4.endTrajectory().fresh().build();
        TrajectoryActionBuilder tab5 = tab4.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(-12, 12), Math.toRadians(180))
                .turnTo(Math.toRadians(-40));
        Action closeOut5 = tab5.endTrajectory().fresh().build();
        // Build the trajectory actions
        Action traj1 = tab1.build();
        Action traj2 = tab2.build();
        Action traj3 = tab3.build();;
        Action traj4 = tab4.build();
        Action traj5 = tab5.build();

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
                        shooter.setupShoot(),
                        new Action() {
                            @Override
                            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                telemetry.addLine("Done shoot");
                                telemetry.update();
                                return false;
                            }
                        },
                        // Sleep actions; ensure sleepAction returns Action
                        new SleepAction(0.5),
                        shooter.shoot(),
                        new SleepAction(1),
                        traj2,
                        closeOut2,

                        // Intake actions; ensure these return Action
                        intake.startIntake(),
                        new Action() {
                            @Override
                            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                telemetry.update();
                                return false;
                            }
                        },
                        new SleepAction(0.500),
                        intake.stopIntake(),
                        new SleepAction(0.500),
                        traj3,
                        closeOut3,
                        new SleepAction(1.000),
                        intake.startIntake(),
                        new SleepAction(4.000),

                        shooter.stop(),
//
                        traj4,
                        closeOut4,
                        new SleepAction(0.500),
                        traj5,
                        closeOut5,
                        // Example shooter action; ensure shoot() returns Action
                        shooter.setupShoot(),
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
                        shooter.shoot(),
                        new SleepAction(1),
                        shooter.stop()
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