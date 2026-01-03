package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config // ===== NEW (Dashboard support) =====
public class ShooterSystem {

    /* =====================================================
       ===== NEW PID DASHBOARD VARIABLES (ADDITIVE) =====
       ===================================================== */
    public static double PID_TARGET_VELOCITY = 3000; // ticks/sec
    public static double PID_kP = 0.00035;
    public static double PID_kI = 0.000001;
    public static double PID_kD = 0.00002;
    public static double PID_TOLERANCE = 40;
    public static double PID_MAX_POWER = 1.0;
    public static int angle = 45;
    /* ===================================================== */

    private static final double DEFAULT_FLYWHEEL_SPEED = 3000; // ticks per second
    private static final double SHOOTER_DEFAULT_ANGLE = Math.PI / 4;

    private final Telemetry telemetry;
    protected DcMotorEx anglerLeft, anglerRight;
    protected DcMotorEx shooterLeft, shooterRight;
    private DistanceSensor blDist, brDist;
    private CRServo bl, br;
    private VisionSystem vision;
    protected final IMU shooterIMU;
    public boolean manualOverride = false;

    private final double ANGLER_SPEED = 0.05;
    private final double ANGLE_TO_TICKS = (1 / 360.0) * 537.6;
    private final double GRAVITY = 9.80665;
    public static final double DELTA_Y = 0.9;
    protected final double SHOOTER_MAX_ANGLE = Math.toRadians(55);
    protected double shootingAngle = -1.0;
    private boolean usingIMU = false;

    /* =====================================================
       ===== NEW PID STATE (ADDITIVE) =====
       ===================================================== */
    private double pidLeftIntegral = 0;
    private double pidRightIntegral = 0;
    private double pidLeftLastError = 0;
    private double pidRightLastError = 0;
    private long pidLastTimeNs = System.nanoTime();
    /* ===================================================== */

    ShooterSystem(HardwareMap hw, Telemetry telemetry, VisionSystem vision, boolean manualOverride) {
        this.vision = vision;
        shooterIMU = hw.get(IMU.class, "shooterIMU");
        this.manualOverride = manualOverride;

        RevHubOrientationOnRobot.LogoFacingDirection logoDir =
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        RevHubOrientationOnRobot.UsbFacingDirection usbDir =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(logoDir, usbDir));
        shooterIMU.initialize(params);

        anglerLeft = hw.get(DcMotorEx.class, "anglerLeft");
        anglerRight = hw.get(DcMotorEx.class, "anglerRight");
        shooterLeft = hw.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hw.get(DcMotorEx.class, "shooterRight");

        blDist = hw.get(DistanceSensor.class, "blDistance");
        brDist = hw.get(DistanceSensor.class, "brDistance");

        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        anglerLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        anglerRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        anglerLeft.setTargetPosition(0);
        anglerRight.setTargetPosition(0);
        anglerLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        anglerRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        anglerRight.setDirection(DcMotorSimple.Direction.REVERSE);
        anglerLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        bl = hw.get(CRServo.class, "inBackLeft");
        br = hw.get(CRServo.class, "inBackRight");
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        this.telemetry = telemetry;
    }

    // Convert ticks/sec -> m/s
    public static double ticksPerSecToMetersPerSec(double ticksPerSec) {
        double ticksPerRev = 28;
        double wheelRadiusMeters = 0.096;
        double revsPerSec = ticksPerSec / ticksPerRev;
        double wheelCircumference = 2.0 * Math.PI * wheelRadiusMeters;
        return revsPerSec * wheelCircumference;
    }

    // meters/sec -> ticks/sec
    public static double metersPerSecToTicksPerSec(double metersPerSec) {
        double ticksPerRev = 120;
        double wheelDiameterMeters = 0.096;
        double wheelCircumference = Math.PI * wheelDiameterMeters;
        return metersPerSec * ticksPerRev / wheelCircumference;
    }

    /* =====================================================
       ===== NEW PID VELOCITY CONTROLLER (ADDITIVE) =====
       ===================================================== */
    public boolean updateShooterPID() {

        long now = System.nanoTime();
        double dt = (now - pidLastTimeNs) / 1e9;
        pidLastTimeNs = now;
        if (dt <= 0) return false;

        double lv = shooterLeft.getVelocity();
        double rv = shooterRight.getVelocity();

        double le = PID_TARGET_VELOCITY - lv;
        double re = PID_TARGET_VELOCITY - rv;

        pidLeftIntegral += le * dt;
        pidRightIntegral += re * dt;

        double ld = (le - pidLeftLastError) / dt;
        double rd = (re - pidRightLastError) / dt;

        pidLeftLastError = le;
        pidRightLastError = re;

        double lp = PID_kP * le + PID_kI * pidLeftIntegral + PID_kD * ld;
        double rp = PID_kP * re + PID_kI * pidRightIntegral + PID_kD * rd;

        lp = Math.max(-PID_MAX_POWER, Math.min(PID_MAX_POWER, lp));
        rp = Math.max(-PID_MAX_POWER, Math.min(PID_MAX_POWER, rp));

        shooterLeft.setPower(lp);
        shooterRight.setPower(rp);
        anglerLeft.setTargetPosition(angle);
        anglerRight.setTargetPosition(angle);

        telemetry.addData("PID Target", PID_TARGET_VELOCITY);
        telemetry.addData("PID Left Vel", lv);
        telemetry.addData("PID Right Vel", rv);

        return Math.abs(le) < PID_TOLERANCE &&
                Math.abs(re) < PID_TOLERANCE;
    }

    /** PID‑only tuning helper */
    public Step.Status setupFlywheelsPIDTuning() {
        return updateShooterPID() ? Step.Status.SUCCESS : Step.Status.RUNNING;
    }

    /* ===================== YOUR ORIGINAL CODE ===================== */

    public Action oneShotAction(Clock clock) {
        return new SequentialSteps(
                new Step() {
                    @Override
                    public Status runStep(TelemetryPacket packet) {
                        AprilTagDetection tag = vision.checkTag();
                        if (tag == null) {
                            return Status.FAILURE;
                        }
                        return Status.SUCCESS;
                    }
                },
                new TimeOutAction(setupFlywheelStepAction(), 2, clock),
                new TimeOutAction(setupAnglerStepAction(), 2, clock),
                new Step() {
                    @Override
                    public Status runStep(TelemetryPacket packet) {
                        telemetry.addLine("before sleep");
                        telemetry.update();
                        return Status.SUCCESS;
                    }
                },
                new SleepStep(0.5),
                new Step() {
                    @Override
                    public Status runStep(TelemetryPacket packet) {
                        telemetry.addLine("after sleep");
                        telemetry.update();
                        return Status.SUCCESS;
                    }
                },
                setupShootStepAction(),
                new SleepStep(3),
                new Step() {
                    @Override
                    public Status runStep(TelemetryPacket packet) {
                        telemetry.addLine("after sleep 3");
                        telemetry.update();
                        return Status.SUCCESS;
                    }
                },
                stopStepAction()
        );
    }

    public Step.Status setupFlywheels() {
        double angularSpeed = DEFAULT_FLYWHEEL_SPEED;
        shootingAngle = SHOOTER_DEFAULT_ANGLE;

        if (!manualOverride) {
            AprilTagDetection tag = vision.checkTag();
            if (tag == null) return Step.Status.FAILURE;

            double targetDistance = getDistanceToTargetMeters(tag);
            this.shootingAngle = calculateShootingAngle(targetDistance);

            double speed = calculateShootingSpeed(targetDistance, shootingAngle);
            if (speed < 0) return Step.Status.FAILURE;

            angularSpeed = metersPerSecToTicksPerSec(speed);
            if (angularSpeed > DEFAULT_FLYWHEEL_SPEED) {
                angularSpeed = DEFAULT_FLYWHEEL_SPEED;
            }
        }

        if (angularSpeed - 10 < shooterLeft.getVelocity() &&
                angularSpeed - 10 < shooterRight.getVelocity()) {
            return Step.Status.SUCCESS;
        }

        shooterLeft.setVelocity(angularSpeed);
        shooterRight.setVelocity(angularSpeed);
        return Step.Status.RUNNING;
    }

    public Step.Status setupAngler() {
        if (manualOverride) shootingAngle = SHOOTER_DEFAULT_ANGLE;

        int ticks = (int) Math.round(ANGLE_TO_TICKS * Math.toDegrees(shootingAngle));
        anglerLeft.setPower(ANGLER_SPEED);
        anglerRight.setPower(ANGLER_SPEED);
        anglerLeft.setTargetPosition(ticks);
        anglerRight.setTargetPosition(ticks);

        if (usingIMU) {
            return Math.abs(
                    Math.abs(shooterIMU.getRobotYawPitchRollAngles()
                            .getPitch(AngleUnit.RADIANS)) - shootingAngle)
                    < Math.toRadians(3)
                    ? Step.Status.SUCCESS
                    : Step.Status.RUNNING;
        }

        return Math.abs(anglerLeft.getTargetPosition()
                - anglerLeft.getCurrentPosition()) < 3
                ? Step.Status.SUCCESS
                : Step.Status.RUNNING;
    }

    public Action setupFlywheelAction() {
        return packet -> setupFlywheels() == Step.Status.RUNNING;
    }

    public Step setupFlywheelStepAction() {
        return packet -> setupFlywheels();
    }

    public Action setupAnglerAction() {
        return packet -> setupAngler() == Step.Status.RUNNING;
    }

    public Step setupAnglerStepAction() {
        return packet -> setupAngler();
    }

    public void shoot() {
        bl.setPower(1);
        br.setPower(1);
    }

    public Action shootAction() {
        return packet -> {
            shoot();
            return false;
        };
    }

    public Step setupShootStepAction() {
        return packet -> {
            shoot();
            return Step.Status.SUCCESS;
        };
    }

    public void stop() {
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        anglerLeft.setPower(0);
        anglerRight.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    public Action stopAction() {
        return packet -> {
            stop();
            return false;
        };
    }

    public Step stopStepAction() {
        return packet -> {
            stop();
            return Step.Status.SUCCESS;
        };
    }

    public double calculateShootingSpeed(double d, double angle) {
        double deltaH = DELTA_Y;
        if (d * Math.tan(angle) - deltaH > 0) {
            return Math.sqrt(
                    (GRAVITY * d * d) /
                            (2.0 * Math.pow(Math.cos(angle), 2) *
                                    (d * Math.tan(angle) - deltaH)));
        }
        return -1.0;
    }

    public boolean checkShootPoosible() {
        AprilTagDetection tag = vision.checkTag();
        return tag != null;
    }

    public double getDistanceToTargetMeters(AprilTagDetection tag) {
        return tag.rawPose.z * 0.0254;
    }

    public double calculateShootingAngle(double targetDistance) {
        double smallerAngle = Math.atan(1 - Math.sqrt(DELTA_Y / targetDistance));
        double steeperAngle = Math.atan(1 + Math.sqrt(DELTA_Y / targetDistance));
        if (steeperAngle < SHOOTER_MAX_ANGLE) return steeperAngle;
        return smallerAngle;
    }
}