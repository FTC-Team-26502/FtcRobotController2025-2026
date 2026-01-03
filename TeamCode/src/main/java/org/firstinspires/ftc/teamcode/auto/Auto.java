//package org.firstinspires.ftc.teamcode.auto;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.acmerobotics.roadrunner.Pose2d;
//
////import org.firstinspires.ftc.teamcode.BaseCodeV3;
//import org.firstinspires.ftc.teamcode.FTC26502OpMode;
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//
///**
// * First version of autonomous code for Decode.
// * Author: dorinamevans@gmail.com
// */
//public abstract class Auto extends FTC26502OpMode {
//    int ballDistMultipliers;
//    int yForBall;
//    public void runOpModeAuto() throws InterruptedException {
//
//        // Define start pose (units must match your RR config; inches are common)
//        Pose2d startPose = new Pose2d(-56, 56, Math.toRadians(-35));
//
//        // Initialize drive with start pose
//        drive = new MecanumDrive(hardwareMap, startPose);
//
//        // Build first trajectory from the start pose
//        TrajectoryActionBuilder driveToShoot = drive.actionBuilder(startPose)
//                .splineToConstantHeading(new Vector2d(-20, 20), Math.toRadians(-45));
//        // Action that ensures pose is set to end of traj1 (optional)
//        Action closeDriveToShoot = driveToShoot.endTrajectory().fresh().build();
//
//        // If you want traj2 to  = driveToShoot.endTrajectory().fresh()
////                .splineToConstantHeadinstart where traj1 ends, start from endTrajectory()
////        TrajectoryActionBuilder tab2g(new Vector2d(-5, 5),  Math.toRadians(-45));
////        Action closeOut2 = tab2.endTrajectory().fresh().build();
//
//        TrajectoryActionBuilder tab3 = driveToShoot.endTrajectory().fresh()
//                .splineToConstantHeading(new Vector2d(0,0), Math.toRadians(-45));
//        Action closeOut3 = tab3.endTrajectory().fresh().build();
//
//        ballDistMultipliers = vision.getObeliskPattern();
//        telemetry.addData("distmultiplyer", ballDistMultipliers);
//
//        TrajectoryActionBuilder ppg = tab3.endTrajectory().fresh()
//                .turnTo(Math.toRadians(180))
//                .splineToConstantHeading(new Vector2d(-65, 12), Math.toRadians(180));
//        Action closeOut4ppg = ppg.endTrajectory().fresh().build();
//        TrajectoryActionBuilder pgp = tab3.endTrajectory().fresh()
//                .turnTo(Math.toRadians(180))
//                .splineToConstantHeading(new Vector2d(-65, -12), Math.toRadians(180));
//        Action closeOut4pgp = pgp.endTrajectory().fresh().build();
//        TrajectoryActionBuilder gpp = tab3.endTrajectory().fresh()
//                .turnTo(Math.toRadians(180))
//                .splineToConstantHeading(new Vector2d(-65, -36), Math.toRadians(180));
//        Action closeOut4gpp = gpp.endTrajectory().fresh().build();
//        TrajectoryActionBuilder tab5gpp = gpp.endTrajectory().fresh()
//                .splineToConstantHeading(new Vector2d(-20, 20), Math.toRadians(180))
//                .turnTo(Math.toRadians(-40));
//        Action closeOut5gpp = tab5gpp.endTrajectory().fresh().build();
//        TrajectoryActionBuilder tab5pgp = pgp.endTrajectory().fresh()
//                .splineToConstantHeading(new Vector2d(-20, 20), Math.toRadians(180))
//                .turnTo(Math.toRadians(-40));
//        Action closeOut5pgp = tab5pgp.endTrajectory().fresh().build();
////        TrajectoryActionBuilder tab5ppg = ppg.endTrajectory().fresh()
////                .splineToConstantHeading(new Vector2d(-20, 20),
////        Action closeOut5ppg = tab5ppg.endTrajectory().fresh().build();
//        // Build the trajectory actions
//        TrajectoryActionBuilder leave = driveToShoot.endTrajectory().fresh()
//                .splineTo(new Vector2d(-90, -20), Math.toRadians(45));
//        Action closeoutleave = leave.endTrajectory().fresh().build();
//        Action driveToShootTraj = driveToShoot.build();
////        Action traj2 = tab2.build();
//        Action traj3 = tab3.       build();
//        Action traj4gpp = gpp.build();
//        Action traj4ppg = ppg.build();
//        Action traj4pgp = pgp.build();
//        Action traj5gpp = tab5gpp.build();
////        Action traj5ppg = tab5ppg.build();
//        Action traj5pgp = tab5pgp.build();
//        Action leavetraj = leave.build();
//
//        // Wait for the start command
//        waitForStart();
//        if (isStopRequested()) return;
//
//        // Run everything sequentially
//        Actions.runBlocking(
//            new SequentialAction(
//                driveToShootTraj,
//                closeDriveToShoot,
//                shooter.setupFlywheelAction(),
//                shooter.setupAnglerAction(),
//                shooter.shootAction(),
//                new SleepAction(1),
//                intake.startIntakeAction(),
//                new SleepAction(5),
//                intake.stopIntakeAction(),
//                new SleepAction(5),
//                new Action() {
//                    @Override
//                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                        telemetry.update();
//                        return false;
//                    }
//                },
//                leavetraj,
//                new Action() {
//                    @Override
//                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                        telemetry.addLine("Traj finished");
//                        telemetry.update();
//                        return false;
//                    }
//                },
//                closeoutleave
//                )
//        );
//    }
//}