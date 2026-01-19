package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.CRServo;


import org.firstinspires.ftc.teamcode.FTC26502OpMode;

import java.util.ArrayList;
import java.util.List;

@Config
public abstract class TeleopWithActions  extends FTC26502OpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    protected Action driveAction = null;
    protected long lastPlanNs = 0;
    protected final double REPLAN_S = 0.12;

    // Action list that is advanced each loop
    private List<Action> runningActions = new ArrayList<>();

    private CRServo servo; // example mechanism for InstantAction

    // Edge detection for buttons so we only enqueue once per press
    private boolean intakeButton1Prev, intakeButton2Prev, closeShootButtonPrev, farShootButtonPrev, headingButtonPrev, rbPrev, backPrev, startPrev;

    private boolean manualDriving = true;

    private boolean intakeSecondRowEnabled = false;
    private boolean intakeFirstRowEnabled = false;
    private boolean shooterEnabled = false;
    private boolean manualOverrideEnabled = false;
    public static int velocity = 1900;
    public static double shootingAngle = 52;
    public static double waitTime = 1.5;
    private boolean bottomShotToggle = true;
    private boolean topShotToggle = true;

    private static boolean pressedOnce(boolean current, boolean previous) {
        return current && !previous;
    }

    public boolean runOpModeTeleop(boolean sagnik) throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {
            boolean intakeButton1 = gamepad1.x;
            boolean intakeButton2 = gamepad1.y;
            boolean closeShootButton = gamepad1.right_bumper;
            boolean farShootButton = gamepad1.left_bumper;
            boolean headingButton = (gamepad1.left_trigger > 0.2);
            if (sagnik){
                 intakeButton1 = gamepad1.a;
                 intakeButton2 = gamepad1.b;
                 closeShootButton = gamepad1.x;
                 farShootButton = gamepad1.y;
                 headingButton = gamepad1.left_bumper;
            }
            TelemetryPacket packet = new TelemetryPacket();
            // 1) Manual driving while actions can run concurrently
            // Robot-centric: left stick (closeShootButton,y), right stick (turn)

            // Keep localization fresh
            drive.updatePoseEstimate();
            // check intake DO NOT chain the controls with else if (makes buttons unreliable and not consistently pressed)

            if (pressedOnce(intakeButton2, intakeButton2Prev)) {
                runningActions.add(intake.secondRow());   // start
            }
            if (pressedOnce(intakeButton1, intakeButton1Prev)) /* Rubber band intake */ {
                runningActions.add(intake.firstRow());
            }

            if (pressedOnce(headingButton,headingButtonPrev)) {
                runningActions.add(shooter.turnToDepot(1));
            }else{
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
                }
            }

            if (gamepad1.left_trigger > 0.75 & gamepad1.right_trigger > 0.75) {
                shooter.manualOverride = true;
            }
            boolean canShoot = shooter.checkShootPoosible();
            sensors.setCanShoot(canShoot);
            sensors.updateIndicatorLights(now());
            if(sensors.isPurpleBall() || sensors.isGreenBall()){
                runningActions.add(intake.ball1Collected());
            }
            if (pressedOnce(closeShootButton, closeShootButtonPrev)) {
                if (bottomShotToggle) {
                    telemetry.addLine("closeShootButton pressed");
                    telemetry.addLine("Can shoot");
                    runningActions.add(shooter.shootBottom(1400, 50, 1));
                    runningActions.add(intake.feedShooter());
                }else{
                    runningActions.add(shooter.stopAction());
                }
                bottomShotToggle = !bottomShotToggle;
            }

            if (pressedOnce(farShootButton, farShootButtonPrev)) {
                if (topShotToggle) {
                    telemetry.addLine("closeShootButton pressed");
                    telemetry.addLine("Can shoot");
                    runningActions.add(shooter.shootTop(velocity,shootingAngle,waitTime));
                }else{
                    runningActions.add(shooter.stopAction());
                }
                topShotToggle = !topShotToggle;
            }
            // TODO add button for manual override

            intakeButton1Prev = intakeButton1;
            intakeButton2Prev = intakeButton2;
            closeShootButtonPrev = closeShootButton;
            farShootButtonPrev = farShootButton;
            headingButtonPrev = headingButton;


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
            packet.put("closeShootButton", pose.position.x);
            packet.put("y", pose.position.y);
            packet.put("Shooter Left Velocity", shooter.shooterLeft.getVelocity());
            packet.put("Shooter Right Velocity", shooter.shooterRight.getVelocity());
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