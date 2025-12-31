package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.CRServo;


import org.firstinspires.ftc.teamcode.FTC26502OpMode;

import java.util.ArrayList;
import java.util.List;


public abstract class TeleopWithActions  extends FTC26502OpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    protected Action driveAction = null;
    protected long lastPlanNs = 0;
    protected final double REPLAN_S = 0.12;

    // Action list that is advanced each loop
    private List<Action> runningActions = new ArrayList<>();

    private CRServo servo; // example mechanism for InstantAction

    // Edge detection for buttons so we only enqueue once per press
    private boolean aPrev, bPrev, xPrev, yPrev, lbPrev, rbPrev, backPrev, startPrev;

    private boolean manualDriving = true;

    private boolean intakeSecondRowEnabled = false;
    private boolean intakeFirstRowEnabled = false;
    private boolean shooterEnabled = false;
    private boolean manualOverrideEnabled = false;

    private static boolean pressedOnce(boolean current, boolean previous) {
        return current && !previous;
    }

    public boolean runOpModeTeleop() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {

            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;
            boolean y = gamepad1.y;
            boolean lb = gamepad1.left_bumper;

            TelemetryPacket packet = new TelemetryPacket();
            // 1) Manual driving while actions can run concurrently
            // Robot-centric: left stick (x,y), right stick (turn)
            double fwd = -gamepad1.left_stick_y;
            double str = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;
            if (manualDriving) {
                //        // Optional slow modes with triggers
                //        double transScale = 1.0 - 0.6 * gamepad1.right_trigger;
                //        double rotScale   = 1.0 - 0.6 * gamepad1.left_trigger;
                //        fwd *= transScale;
                //        str *= transScale;
                //        turn *= rotScale;
                drive.setDrivePowers(fwd, str, turn);

            } else {
                // patching trajectories
                // Advance any running action
                if (driveAction != null && !driveAction.run(packet)) {
                    driveAction = null;
                }
                // Periodic replan from sticks
                long now = System.nanoTime();
                if ((now - lastPlanNs) * 1e-9 >= REPLAN_S) {
                    try {
                        Action micro = drive.buildDriverNudgeAction(str, fwd, turn);
                        lastPlanNs = now;
                        if (micro != null) {
                            driveAction = micro; // replace for responsiveness
                        }
                    } catch (Exception exception) {
                        // any path violations will not stop the loop
                        driveAction = null;
                        drive.setDrivePowers(str, fwd, turn);
                    }
                }
            }
            // Keep localization fresh
            drive.updatePoseEstimate();
            // check intake DO NOT chain the controls with else if (makes buttons unreliable and not consistently pressed)

            if (pressedOnce(b, bPrev)) {
                runningActions.add(intake.secondRow());   // start
            }
            if (pressedOnce(a, aPrev)) /* Rubber band intake */ {
                runningActions.add(intake.firstRow());
            }

            if (lb) {
                runningActions.add(intake.stallDetected());
            }

            if (gamepad1.left_trigger > 0.75 & gamepad1.right_trigger > 0.75) {
                shooter.manualOverride = true;
            }
            boolean canShoot = shooter.checkShootPoosible();
            sensors.setCanShoot(canShoot);
            sensors.updateIndicatorLights(now());
            if (pressedOnce(x, xPrev)) {
                telemetry.addLine("x pressed");
                telemetry.addLine("Can shoot");
                runningActions.add(shooter.oneShotAction(this));

            }

            if (pressedOnce(y, yPrev)) {
                runningActions.add(shooter.stopAction());
            }
            // TODO add button for manual override

            aPrev = a;
            bPrev = b;
            xPrev = x;
            yPrev = y;
            lbPrev = lb;


            // 3) Advance running actions
            List<Action> stillRunning = new ArrayList<>();
            for (Action action : runningActions) {
                // run returns true if it should be kept
                if (action.run(packet)) {
                    stillRunning.add(action);
                }
            }
            runningActions = stillRunning;

            // 4) Dashboard telemetry
            Pose2d pose = drive.localizer.getPose();
            packet.put("x", pose.position.x);
            packet.put("y", pose.position.y);
            packet.put("heading_deg", Math.toDegrees(pose.heading.toDouble()));
            packet.put("running_actions", runningActions.size());

            dash.sendTelemetryPacket(packet);

            // Update previous states for edge detection


            rbPrev = gamepad1.right_bumper;
            backPrev = gamepad1.back;
            startPrev = gamepad1.start;
            dash.sendTelemetryPacket(packet);
        }

        return false;
    }

}