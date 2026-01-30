package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "MecanumCurrentDashboardOnly")
public class MotorCurrentTest extends OpMode {

    // Motors
    DcMotorEx fl, fr, bl, br;

    // Dashboard telemetry
    Telemetry dashboardTelemetry;

    // Max current per motion
    double maxForward = 0;
    double maxBackward = 0;
    double maxStrafeLeft = 0;
    double maxStrafeRight = 0;
    double maxTurnLeft = 0;
    double maxTurnRight = 0;

    // Motion label
    String motion = "STOP";

    @Override
    public void init() {
        fl = hardwareMap.get(DcMotorEx.class, "leftFront");
        fr = hardwareMap.get(DcMotorEx.class, "leftBack");
        bl = hardwareMap.get(DcMotorEx.class, "rightBack");
        br = hardwareMap.get(DcMotorEx.class, "rightFront");
        // Hardware mapping

        dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
    }

    @Override
    public void loop() {

        // ======================
        // GAMEPAD INPUT
        // ======================
        double forward = -gamepad1.left_stick_y;
        double strafe  =  gamepad1.left_stick_x;
        double turn    =  gamepad1.right_stick_x;

        // Deadzone
        if (Math.abs(forward) < 0.05) forward = 0;
        if (Math.abs(strafe)  < 0.05) strafe  = 0;
        if (Math.abs(turn)    < 0.05) turn    = 0;

        // ======================
        // MECANUM DRIVE
        // ======================
        double flPower = forward + strafe + turn;
        double frPower = forward - strafe - turn;
        double blPower = forward - strafe + turn;
        double brPower = forward + strafe - turn;

        double maxPower = Math.max(1.0, Math.max(
                Math.abs(flPower),
                Math.max(Math.abs(frPower),
                        Math.max(Math.abs(blPower), Math.abs(brPower)))
        ));

        fl.setPower(flPower / maxPower);
        fr.setPower(frPower / maxPower);
        bl.setPower(blPower / maxPower);
        br.setPower(brPower / maxPower);

        // ======================
        // MOTOR CURRENT (worst motor)
        // ======================
        double current = Math.max(
                Math.max(fl.getCurrent(CurrentUnit.AMPS),
                        fr.getCurrent(CurrentUnit.AMPS)),
                Math.max(bl.getCurrent(CurrentUnit.AMPS),
                        br.getCurrent(CurrentUnit.AMPS))
        );

        // ======================
        // MOTION CLASSIFICATION
        // ======================
        motion = "STOP";

        if (Math.abs(forward) > 0.1 &&
                Math.abs(strafe)  < 0.1 &&
                Math.abs(turn)    < 0.1) {

            motion = forward > 0 ? "FORWARD" : "BACKWARD";
        }
        else if (Math.abs(strafe) > 0.1 &&
                Math.abs(forward) < 0.1 &&
                Math.abs(turn) < 0.1) {

            motion = strafe > 0 ? "STRAFE_RIGHT" : "STRAFE_LEFT";
        }
        else if (Math.abs(turn) > 0.1 &&
                Math.abs(forward) < 0.1 &&
                Math.abs(strafe) < 0.1) {

            motion = turn > 0 ? "TURN_RIGHT" : "TURN_LEFT";
        }

        // ======================
        // TRACK MAX CURRENT
        // ======================
        switch (motion) {
            case "FORWARD":
                maxForward = Math.max(maxForward, current);
                break;
            case "BACKWARD":
                maxBackward = Math.max(maxBackward, current);
                break;
            case "STRAFE_LEFT":
                maxStrafeLeft = Math.max(maxStrafeLeft, current);
                break;
            case "STRAFE_RIGHT":
                maxStrafeRight = Math.max(maxStrafeRight, current);
                break;
            case "TURN_LEFT":
                maxTurnLeft = Math.max(maxTurnLeft, current);
                break;
            case "TURN_RIGHT":
                maxTurnRight = Math.max(maxTurnRight, current);
                break;
        }

        // ======================
        // RESET MAX VALUES
        // ======================
        if (gamepad1.a) {
            maxForward = maxBackward = 0;
            maxStrafeLeft = maxStrafeRight = 0;
            maxTurnLeft = maxTurnRight = 0;
        }

        // ======================
        // DRIVER STATION TELEMETRY
        // ======================
        telemetry.addData("Motion", motion);
        telemetry.addData("Current (A)", "%.2f", current);
        telemetry.addData("Max Forward", "%.2f", maxForward);
        telemetry.addData("Max Backward", "%.2f", maxBackward);
        telemetry.addData("Max Strafe Left", "%.2f", maxStrafeLeft);
        telemetry.addData("Max Strafe Right", "%.2f", maxStrafeRight);
        telemetry.addData("Max Turn Left", "%.2f", maxTurnLeft);
        telemetry.addData("Max Turn Right", "%.2f", maxTurnRight);
        telemetry.addLine("Press A to reset max values");
        telemetry.update();

        // ======================
        // DASHBOARD TELEMETRY
        // ======================
        dashboardTelemetry.addData("current", current);
        dashboardTelemetry.addData("motion", motion);

        dashboardTelemetry.addData("maxForward", maxForward);
        dashboardTelemetry.addData("maxBackward", maxBackward);
        dashboardTelemetry.addData("maxStrafeLeft", maxStrafeLeft);
        dashboardTelemetry.addData("maxStrafeRight", maxStrafeRight);
        dashboardTelemetry.addData("maxTurnLeft", maxTurnLeft);
        dashboardTelemetry.addData("maxTurnRight", maxTurnRight);
        dashboardTelemetry.update();
    }
}