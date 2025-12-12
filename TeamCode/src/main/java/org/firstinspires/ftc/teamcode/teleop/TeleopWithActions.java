package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.BaseCodeV3;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TimeOutAction;

import java.util.ArrayList;
import java.util.List;

public abstract class TeleopWithActions  extends BaseCodeV3 {
    private FtcDashboard dash = FtcDashboard.getInstance();
    protected Action driveAction = null;
    protected long lastPlanNs = 0;
    protected final double REPLAN_S = 0.12;

    // Action list that is advanced each loop
    private List<Action> runningActions = new ArrayList<>();

    // Drive + optional mechanism example
    private MecanumDrive drive;
    private CRServo servo; // example mechanism for InstantAction

    // Edge detection for buttons so we only enqueue once per press
    private boolean aPrev, bPrev, xPrev, yPrev, lbPrev, rbPrev, backPrev, startPrev;

    private boolean manualDriving = true;

    public void runOpModeTeleop() throws InterruptedException {

        while (opModeIsActive()) {

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
                drive.setDrivePowers(str, fwd, turn);

            } else  {
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
            // check intake
            if (pressedOnce(gamepad1.a, aPrev)) {
                // stop the shooter if we intake
                runningActions.add(shooter.stopAction());
                runningActions.add(intake.startIntakeAction());
            } else if (pressedOnce(gamepad1.b, bPrev)) {
                runningActions.add(intake.stopIntakeAction());
            }
            boolean canShoot = shooter.checkShootPoosible();
            sensors.setCanShoot(canShoot);
            sensors.updateIndicatorLights(now());
            if (pressedOnce(gamepad1.x, xPrev)) {
                if (canShoot) {
                    runningActions.add(shooter.oneShotAction(this));
                }
            } else if (pressedOnce(gamepad1.y, yPrev)) {
                runningActions.add(shooter.stopAction());
            }
            // TODO add button for manual override



            // 3) Advance running actions
            List<Action> stillRunning = new ArrayList<>();
            for (Action action : runningActions) {
                // draw previews for active actions
                action.preview(packet.fieldOverlay());

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
            aPrev = gamepad1.a;
            bPrev = gamepad1.b;
            xPrev = gamepad1.x;
            yPrev = gamepad1.y;
            lbPrev = gamepad1.left_bumper;
            rbPrev = gamepad1.right_bumper;
            backPrev = gamepad1.back;
            startPrev = gamepad1.start;
            dash.sendTelemetryPacket(packet);
        }
    }


    private static boolean pressedOnce(boolean current, boolean previous) {
        return current && !previous;
    }

}


