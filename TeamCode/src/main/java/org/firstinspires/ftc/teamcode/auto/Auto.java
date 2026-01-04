package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;

//import org.firstinspires.ftc.teamcode.BaseCodeV3;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FTC26502OpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

/**
 * First version of autonomous code for Decode.
 * Author: dorinamevans@gmail.com
 */
public abstract class Auto extends FTC26502OpMode {
    int ballDistMultipliers;
    public int y;
    public int x;
    public void runOpModeAuto() throws InterruptedException {
        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        telemetry.addLine("Init complete. Adjust variables in Dashboard > Config.");
        telemetry.update();
        // Define start pose (units must match your RR config; inches are common)
        Pose2d startPose = new Pose2d(-56, 56, Math.toRadians(-35));

        // Initialize drive with start pose
        drive = new MecanumDrive(hardwareMap, startPose);

        // Build first trajectory from the start pose
        TrajectoryActionBuilder driveToShoot = drive.actionBuilder(startPose)
                .splineToConstantHeading(new Vector2d(x, y), Math.toRadians(-45));
        // Action that ensures pose is set to end of traj1 (optional)
        Action closeDriveToShoot = driveToShoot.endTrajectory().fresh().build();

        Action driveToShootTraj = driveToShoot.build();

        // Wait for the start command
        waitForStart();
        if (isStopRequested()) return;

        // Run everything sequentially
        Actions.runBlocking(
            new SequentialAction(
                driveToShootTraj,
                closeDriveToShoot,
                shooter.setupFlywheelAction(),
                shooter.setupAnglerAction(),
                shooter.shootAction(),
                new SleepAction(1),
                intake.startIntakeAction(),
                new SleepAction(5),
                intake.stopIntakeAction(),
                new SleepAction(5),
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        telemetry.update();
                        return false;
                    }
                }
            )
        );
    }
}