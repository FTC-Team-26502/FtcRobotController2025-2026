package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
@Config
public abstract class Auto extends FTC26502OpMode {
    public static int y = 20;
    public static int x = -20;
    public static int heading = -45;
    public static int angle = 50;
    public static int power = 900;
    public static int wait = 1;
    public static int y1 = 60;
    public static int x1 = -25;
    public static int heading1 = -45;
    public void runOpModeAuto() throws InterruptedException {
        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

        telemetry.addLine("Init complete. Adjust variables in Dashboard > Config.");
        telemetry.update();
        // Define start pose (units must match your RR config; inches are common)
        Pose2d startPose = new Pose2d(-56, -56, Math.toRadians(45));

        // Initialize drive with start pose
        drive = new MecanumDrive(hardwareMap, startPose);

        // Build first trajectory from the start pose
        TrajectoryActionBuilder driveToShoot = drive.actionBuilder(startPose)
                .splineTo(new Vector2d(x, y), Math.toRadians(heading));
        // Action that ensures pose is set to end of traj1 (optional)
        Action closeDriveToShoot = driveToShoot.endTrajectory().fresh().build();
        TrajectoryActionBuilder pickup1 = drive.actionBuilder(new Pose2d(x,y, Math.toRadians(heading)))
                .splineTo(new Vector2d(x1, y1), Math.toRadians(heading1));
        // Action that ensures pose is set to end of traj1 (optional)
        Action closePickup1 = pickup1.endTrajectory().fresh().build();
        Action driveToShootTraj = driveToShoot.build();
        Action pickup1Traj = pickup1.build();

        // Wait for the start command
        waitForStart();
        if (isStopRequested()) return;

        // Run everything sequentially
        Actions.runBlocking(
            new SequentialAction(
                driveToShootTraj,
                closeDriveToShoot,
//                new Action() {
//                    @Override
//                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                        telemetry.addLine("prefire");
//                        telemetry.update();
//                        return false;
//                    }
//                },
//                shooter.shootingBottomTriangleAuto(angle, power),
//                new Action() {
//                    @Override
//                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                        telemetry.addLine("Shoot 1 done");
//                        telemetry.update();
//                        return false;
//                    }
//                },
//                new SleepAction(wait),
//                new Action() {
//                    @Override
//                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                        telemetry.addLine("waited");
//                        telemetry.update();
//                        return false;
//                    }
//                },
                intake.startIntakeAction(),
//                new SleepAction(wait),
//                new Action() {
//                    @Override
//                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                        telemetry.addLine("intake started");
//                        return false;
//                    }
//                },
                pickup1Traj,
                closePickup1,
                new SleepAction(wait),
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