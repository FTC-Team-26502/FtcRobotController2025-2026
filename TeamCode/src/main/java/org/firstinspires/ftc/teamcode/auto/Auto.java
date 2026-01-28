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
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * First version of autonomous code for Decode.
 * Author: dorinamevans@gmail.com
 */
@Config
public abstract class Auto extends FTC26502OpMode {
    public static int y2 = -20;
    public static int y = -10;
    public static int x = -10;
    public static int heading = 45;
    public static int angle = 50;
    public static int power = 1350;
    public static double wait = 1.5;
    public static int wait1 = 2;
    public static int wait2 = 1;
    public static int wait3 = 2;
    public static int y1 = 72;
    public static int x1 = 18;
    public static int x2 = 56;
    public static int heading1 = 45;
    public static int yMultiplier;

    protected ElapsedTime runtime = new ElapsedTime();

    public void runOpModeAuto() throws InterruptedException {
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
        Pose2d startPose = new Pose2d(-56, -56*yMultiplier, Math.toRadians(45*yMultiplier));

        // Initialize drive with start pose
        drive = new MecanumDrive(hardwareMap, startPose);

        // Build first trajectory from the start pose
        TrajectoryActionBuilder driveToShoot = drive.actionBuilder(startPose)
                .splineTo(new Vector2d(x, y*yMultiplier), Math.toRadians(heading*yMultiplier));
        // Action that ensures pose is set to end of traj1 (optional)
        Action closeDriveToShoot = driveToShoot.endTrajectory().fresh().build();

        TrajectoryActionBuilder driveToFirstRow = drive.actionBuilder(new Pose2d(x,y*yMultiplier, Math.toRadians(heading*yMultiplier)))
                .splineTo(new Vector2d(-12,-50*yMultiplier), Math.toRadians(-90*yMultiplier))
                .lineToY(-y1*yMultiplier);
        Action closeFirstRowAndShoot = driveToFirstRow.endTrajectory().fresh().build();

        TrajectoryActionBuilder shootAfterFirstRow = drive.actionBuilder(new Pose2d(-12,-70*yMultiplier, Math.toRadians(-90*yMultiplier)))
                .lineToY(y2*yMultiplier)
                .turnTo(Math.toRadians(heading*yMultiplier));
        Action closeShootAfterFirstRow = shootAfterFirstRow.endTrajectory().fresh().build();

        TrajectoryActionBuilder secondRowAndShoot = drive.actionBuilder(new Pose2d(x,y*yMultiplier, Math.toRadians(heading*yMultiplier)))
                .splineTo(new Vector2d(x1,-30*yMultiplier), Math.toRadians(-90*yMultiplier))
                .splineToConstantHeading(new Vector2d(x1,-y1*yMultiplier), Math.toRadians(-90*yMultiplier))
                .splineToConstantHeading(new Vector2d(x1,-60*yMultiplier), Math.toRadians(-90*yMultiplier))
                .splineTo(new Vector2d(-12,-12*yMultiplier),Math.toRadians(45*yMultiplier));
        Action closeSecondRowAndShoot = secondRowAndShoot.endTrajectory().fresh().build();

        TrajectoryActionBuilder thirdRowAndShoot = drive.actionBuilder(new Pose2d(x,y*yMultiplier, Math.toRadians(heading*yMultiplier)))
                .splineTo(new Vector2d(x2,-30*yMultiplier), Math.toRadians(-90*yMultiplier))
                .splineToConstantHeading(new Vector2d(x2,-70*yMultiplier), Math.toRadians(-90*yMultiplier))
                .splineToConstantHeading(new Vector2d(x2,-60*yMultiplier), Math.toRadians(-90*yMultiplier))
                .splineTo(new Vector2d(-12,-12*yMultiplier),Math.toRadians(45*yMultiplier));
        Action closeThridRowAndShoot = thirdRowAndShoot.endTrajectory().fresh().build();

        TrajectoryActionBuilder leave = drive.actionBuilder(new Pose2d(x,y*yMultiplier, Math.toRadians(heading*yMultiplier)))
                .splineTo(new Vector2d(0,-72*yMultiplier),Math.toRadians(0*yMultiplier));
        Action closeLeave = leave.endTrajectory().fresh().build();

        // Action that ensures pose is set to end of traj1 (optional)
        Action driveToShootTraj = driveToShoot.build();
        Action driveToFirstRowTraj = driveToFirstRow.build();
        Action shootAfterFirstRowTraj = shootAfterFirstRow.build();

        Action secondRowAndShootTraj = secondRowAndShoot.build();
        Action thirdRowAndShootTraj = thirdRowAndShoot.build();
        Action leaveTraj = leave.build();

        // Wait for the start command
        waitForStart();
        runtime.reset();
        if (isStopRequested()) return;

        Action autoAction = new SequentialAction(
                new StoppableAction(driveToShootTraj),
                new StoppableAction(closeDriveToShoot),
                new StoppableAction(
                        new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        telemetry.addLine("prefire");
                        telemetry.update();
                        return false;
                    }
                }),
                new StoppableAction(shooter.shootBottom(power, angle, 1)),
                new StoppableAction(new SleepAction(wait)),
                new StoppableAction(intake.startIntakeAction()),
                new StoppableAction(new SleepAction(wait1)),//,
                new StoppableAction(shooter.stopAction()),
                new StoppableAction(driveToFirstRowTraj),
                new StoppableAction(closeFirstRowAndShoot),
                new StoppableAction(new SleepAction(wait2)),
                new StoppableAction(intake.stopIntakeAction()),
                new StoppableAction(shootAfterFirstRowTraj),
                new StoppableAction(closeShootAfterFirstRow),
                new StoppableAction(shooter.shootBottom(power, angle, 1)),
                new StoppableAction(new SleepAction(wait)),
                new StoppableAction(intake.startIntakeAction()),
                new StoppableAction(new SleepAction(wait3)),//,
//                new StoppableAction(shooter.stopAction()),
//                new StoppableAction(secondRowAndShootTraj),
//                new StoppableAction(closeSecondRowAndShoot),
//                new StoppableAction(intake.stopIntakeAction()),
//                new StoppableAction(shooter.shootBottom(power, angle, 1)),
//                new StoppableAction(new SleepAction(wait)),
//                new StoppableAction(intake.startIntakeAction()),
//                new StoppableAction(new SleepAction(5)),
//                new StoppableAction(shooter.stopAction()),
                new StoppableAction(shooter.dropShooter()),
                new StoppableAction(leaveTraj),
                new StoppableAction(closeLeave)
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


        );

        try {
            // Run everything sequentially
            Actions.runBlocking(autoAction);
        } finally {
            shooter.shutdown();
        }

    }

    class StoppableAction implements Action {

        public Action inner;

        StoppableAction( Action inner) {
            this.inner = inner;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if ( runtime.milliseconds() > 24000 || !opModeIsActive() || isStopRequested() ) {
                return false;
            }
            if ( inner instanceof SleepAction ) {
                if ( 25 - runtime.seconds() - ((SleepAction)inner).getDt() <= 1 ) {
                    return false;
                }
            }
            return inner.run(telemetryPacket);
        }
    }

}