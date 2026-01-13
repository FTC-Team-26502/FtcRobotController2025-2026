package org.firstinspires.ftc.teamcode.opmodes.shooter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Test", group = "Shooter")
public class ShotterPID extends LinearOpMode {

    // ========= USER CONFIG =========
    // Hardware names
    private static final String LEFT_NAME  = "leftShooter";
    private static final String RIGHT_NAME = "rightShooter";
    // Optional feeder example:
    // private static final String FEEDER_NAME = "feeder";

    // Encoder ticks per revolution at measured shaft (integrated)
    private static final double TICKS_PER_REV = 28.0;

    // RPM presets tuned for perforated plastic balls with ~3" flywheels
    private static final double RPM_NEAR = 3600.0;
    private static final double RPM_12FT = 3900.0; // start here for 10–12 ft; tune ±100–200
    private static final double RPM_FAR  = 4200.0;

    // Ready-to-fire tolerances (stricter for plastic balls)
    private static final double READY_TOL_RPM = 70.0; // each wheel within ± this of target
    private static final double DELTA_TOL_RPM = 25.0; // |L−R| must be ≤ this at feed time
    private static final long   READY_HOLD_MS = 60;    // must stay in-band this long

    // Velocity PIDF (RUN_USING_ENCODER) — starting points for direct drive
    private static final double P = 0.032;
    private static final double I = 0.000;
    private static final double D = 0.0022;
    private static final double F = 0.00030; // raise/lower to get <1% steady-state error

    // Cross-coupling gain to match left/right speeds
    private static final double K_VEL = 0.16; // tune 0.14–0.20 for plastic balls

    // Shot recovery boost (plastic balls load the wheels harder)
    private static final double RECOVERY_BOOST_PCT = 0.06; // +6% target after shot
    private static final long   RECOVERY_MS        = 200;  // duration

    // Require trigger to release between shots (edge-triggered feed)
    private static final boolean REQUIRE_TRIGGER_RISE = true;
    // ========= END USER CONFIG =========

    private DcMotorEx left, right;
    // Example feeder:
    // private DcMotorEx feeder;

    private double targetRpm = RPM_12FT;
    private double baseTargetTps = 0.0;
    private double tolTps;
    private double deltaTolTps;
    private long boostUntilMs = 0;
    private long readySinceMs = Long.MAX_VALUE;
    private boolean triggerPrev = false;

    @Override
    public void runOpMode() {
        // Map hardware
        left  = hardwareMap.get(DcMotorEx.class, LEFT_NAME);
        right = hardwareMap.get(DcMotorEx.class, RIGHT_NAME);
        // feeder = hardwareMap.get(DcMotorEx.class, FEEDER_NAME);

        // Ensure both spin "forward" toward the exit; one usually needs REVERSE
        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE); // flip if needed for your build

        // Encoders + velocity mode
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // PIDF
        PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, F);
        left.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        right.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        // Optional: zero power behaviors
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Feeder example
        // feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize targets/tolerances
        setTargetRpm(targetRpm);
        tolTps = rpmToTps(READY_TOL_RPM);
        deltaTolTps = rpmToTps(DELTA_TOL_RPM);

        telemetry.addLine("Dual Flywheel Shooter (Plastic Balls)");
        telemetry.addData("Preset", presetName(targetRpm));
        telemetry.addData("Target RPM", "%.0f", targetRpm);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read current velocities (ticks/sec)
            double vL = left.getVelocity();
            double vR = right.getVelocity();

            // Base target with temporary recovery boost
            long now = System.currentTimeMillis();
            double targetTps = baseTargetTps;
            if (now < boostUntilMs) {
                targetTps *= (1.0 + RECOVERY_BOOST_PCT);
            }

            // Cross-coupled setpoints to keep L/R matched
            double eDiff = vL - vR; // positive if left is faster
            double leftTargetTps  = targetTps - K_VEL * eDiff;
            double rightTargetTps = targetTps + K_VEL * eDiff;

            left.setVelocity(leftTargetTps);
            right.setVelocity(rightTargetTps);

            // Ready-to-fire gating
            boolean inBandL = Math.abs(vL - targetTps) <= tolTps;
            boolean inBandR = Math.abs(vR - targetTps) <= tolTps;
            boolean matched = Math.abs(vL - vR) <= deltaTolTps;
            boolean readyBasic = inBandL && inBandR && matched;

            if (readyBasic) {
                if (readySinceMs == Long.MAX_VALUE) readySinceMs = now;
            } else {
                readySinceMs = Long.MAX_VALUE;
            }
            boolean readyToFire = readyBasic && (now - readySinceMs >= READY_HOLD_MS);

            // Preset adjustments (gamepad1)
            if (gamepad1.a) setTargetRpm(RPM_12FT);  // main preset (~12 ft)
            if (gamepad1.x) setTargetRpm(RPM_NEAR);  // near
            if (gamepad1.b) setTargetRpm(RPM_FAR);   // far
            if (gamepad1.dpad_up)   setTargetRpm(targetRpm + 50);          // fine up
            if (gamepad1.dpad_down) setTargetRpm(Math.max(1000, targetRpm - 50)); // fine down

            // Trigger and edge detection for feeding
            boolean trigger = gamepad1.right_bumper;
            boolean allowTrigger = !REQUIRE_TRIGGER_RISE || (trigger && !triggerPrev);
            boolean feedCmd = readyToFire && allowTrigger;

            // Detect shot or heavy dip to apply recovery boost
            boolean shotTriggered = feedCmd;
            boolean heavyDip = (vL < targetTps - 3 * tolTps) || (vR < targetTps - 3 * tolTps);
            if (shotTriggered || heavyDip) {
                boostUntilMs = now + RECOVERY_MS;
            }
            triggerPrev = trigger;

            // Drive feeder here (example: run while ready and bumper event occurs)
            // double feederPower = feedCmd ? 1.0 : 0.0;
            // feeder.setPower(feederPower);

            // Telemetry
            telemetry.addData("Preset", presetName(targetRpm));
            telemetry.addData("Target RPM", "%.0f", targetRpm);
            telemetry.addData("Target tps", "%.0f", baseTargetTps);
            telemetry.addData("Left tps / Right tps", "%.0f / %.0f", vL, vR);
            telemetry.addData("Δv (tps)", "%.0f", (vL - vR));
            telemetry.addData("Ready", readyToFire);
            telemetry.addData("In-band ±RPM", READY_TOL_RPM);
            telemetry.addData("ΔRPM limit", DELTA_TOL_RPM);
            telemetry.update();
        }

        // Stop motors
        left.setPower(0);
        right.setPower(0);
        // feeder.setPower(0);
    }

    private void setTargetRpm(double rpm) {
        targetRpm = rpm;
        baseTargetTps = rpmToTps(rpm);
    }

    private static double rpmToTps(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    @SuppressWarnings("unused")
    private static double tpsToRpm(double tps) {
        return tps * 60.0 / TICKS_PER_REV;
    }

    private String presetName(double rpm) {
        if (Math.abs(rpm - RPM_12FT) < 1) return "12ft";
        if (Math.abs(rpm - RPM_NEAR) < 1) return "Near";
        if (Math.abs(rpm - RPM_FAR)  < 1) return "Far";
        return "Custom";
    }
}