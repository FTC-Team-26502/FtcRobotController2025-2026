package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@Config
public class ShooterSystem {
    private static final double DEFAULT_FLYWHEEL_SPEED = 3000; // ticks per second
    private static final double SHOOTER_DEFAULT_ANGLE = Math.PI/4;
    private final Telemetry telemetry;
    public Servo anglerLeft, anglerRight;
    public DcMotorEx shooterLeft;
    public DcMotorEx shooterRight;
    private DistanceSensor blDist, brDist;
    private CRServo bl, br;
    protected VisionSystem vision;
    protected MecanumDrive drive;
    protected final IMU shooterIMU;
    public boolean manualOverride = false;

    private final double ANGLER_SPEED = 0.05;
    private final double STARTING_ANGLE = 45.0;
    private final double ANGLE_TO_TICKS = (1/360.0) * 1425;
    private final double GRAVITY = 9.80665;
    public static final double DELTA_Y = 0.9;
    protected final double SHOOTER_MAX_ANGLE = Math.toRadians(55);
    protected double shootingAngle = -1.0;
    private boolean usingIMU = false;
    private double x;
    protected double turnPower;

    protected static double P = 0.032;
    protected static double I = 0.000;
    protected static double D = 0.0022;
    protected static double FLeft = 11.7;
    protected static double FRight = 11.7;


    private static final double RPM_NEAR = 3600.0;
    private static final double RPM_12FT = 3900.0; // start here for 10–12 ft; tune ±100–200
    private static final double RPM_FAR  = 4200.0;

    // Ready-to-fire tolerances (stricter for plastic balls)
    private static final double READY_TOL_RPM = 70.0; // each wheel within ± this of target
    private static final double DELTA_TOL_RPM = 25.0; // |L−R| must be ≤ this at feed time
    private static final long   READY_HOLD_MS = 60;    // must stay in-band this long


    // Cross-coupling gain to match left/right speeds
    private static final double K_VEL_LEFT = 0.16; // tune 0.14–0.20 for plastic ballsspeeds
    private static final double K_VEL_RIGHT = 0.16; // tune 0.14–0.20 for plastic balls

    // Shot recovery boost (plastic balls load the wheels harder)
    private static final double RECOVERY_BOOST_PCT = 0.06; // +6% target after shot
    private static final long   RECOVERY_MS        = 200;  // duration

    // Require trigger to release between shots (edge-triggered feed)
    private static final boolean REQUIRE_TRIGGER_RISE = true;
    // ========= END USER CONFIG =========

    private double targetRpm = RPM_12FT;
    private double tolTps = 50;
    private double deltaTolTps = 25;
    private long boostUntilMs = 0;
    private long readySinceMs = Long.MAX_VALUE;
    private boolean triggerPrev = false;

    private boolean toggleShootTop = false;

    private boolean toggleShootBottom = false;

    private boolean toggleHeadingAdjust = false;

    ShooterSystem(HardwareMap hw, Telemetry telemetry, VisionSystem vision, MecanumDrive drive, boolean manualOverride) {
        this.vision = vision;
        this.drive = drive;
        shooterIMU = hw.get(IMU.class, "shooterIMU");
        this.manualOverride = manualOverride;

        // Set how the IMU is mounted on the robot. Update these to match your physical mounting.
        RevHubOrientationOnRobot.LogoFacingDirection logoDir =
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN;      // change as needed
        RevHubOrientationOnRobot.UsbFacingDirection usbDir =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;  // change as needed
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(logoDir, usbDir));
        shooterIMU.initialize(params);
        anglerLeft = hw.get(Servo.class, "anglerLeft");
        anglerRight = hw.get(Servo.class, "anglerRight");
        shooterLeft = hw.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hw.get(DcMotorEx.class, "shooterRight");

        blDist = hw.get(DistanceSensor.class, "blDistance");
        brDist = hw.get(DistanceSensor.class, "brDistance");

        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        anglerLeft.setPosition(0);
        anglerRight.setPosition(0);

        PIDFCoefficients pidfLeft = new PIDFCoefficients(P, I, D, FRight);
        PIDFCoefficients pidfRight = new PIDFCoefficients(P, I, D, FLeft);
        shooterLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfLeft);
        shooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfRight);

        anglerRight.setDirection(Servo.Direction.REVERSE);
        anglerLeft.setDirection(Servo.Direction.FORWARD);

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

    // Convenience: meters/sec -> ticks/sec (inverse)
    public static double metersPerSecToTicksPerSec(double metersPerSec) {
        double ticksPerRev = 120;
        //Wheel circumference C = π × D
        double wheelDiameterMeters = 0.096;
        double wheelCircumference = Math.PI * wheelDiameterMeters;
        return metersPerSec * ticksPerRev / wheelCircumference;
    }

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
                new TimeOutAction(
                        setupAnglerStepAction(), 2, clock),
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
    public Action dropShooter(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                anglerLeft.setPosition(0);
                anglerRight.setPosition(0);
                return false;
            }
        };
    }
    public Step.Status setupFlywheels() {
        double angularSpeed = DEFAULT_FLYWHEEL_SPEED; //ticks/sec
        shootingAngle = SHOOTER_DEFAULT_ANGLE;
        if (!manualOverride) {

            AprilTagDetection tag = vision.checkTag();
            if (tag == null) {
                return Step.Status.FAILURE;
            }
            double tagDistance = getDistanceToTargetMeters(tag);
            telemetry.addData( "tag distance ", tagDistance );
//            double targetDistance = selectiveAverage(tagDistance,
//                    blDist.getDistance(DistanceUnit.METER),
//                    brDist.getDistance(DistanceUnit.METER));
            double targetDistance = tagDistance;
            this.shootingAngle = calculateShootingAngle(targetDistance);
            telemetry.addLine(String.format("target shooting angle %.2f", Math.toDegrees(shootingAngle)));
            double speed = calculateShootingSpeed(targetDistance, shootingAngle); // mps
            if (speed < 0) {
                return Step.Status.FAILURE;
            }
            angularSpeed = metersPerSecToTicksPerSec(speed);
            if (angularSpeed > DEFAULT_FLYWHEEL_SPEED) {
                angularSpeed = DEFAULT_FLYWHEEL_SPEED;
            }
            telemetry.addLine(String.format("target %2f left %2f right %2f", angularSpeed,
                    shooterLeft.getVelocity(), shooterRight.getVelocity()));
        }
        if ( angularSpeed - 10 < shooterLeft.getVelocity() &&
                angularSpeed -10 < shooterRight.getVelocity()) {
            telemetry.update();
            return Step.Status.SUCCESS;
        }
        shooterLeft.setVelocity(angularSpeed);
        shooterRight.setVelocity(angularSpeed);
        return Step.Status.RUNNING;
    }

    public Step.Status setupAngler() {
        telemetry.addLine("setup angler called");
        if (manualOverride) {
            shootingAngle = SHOOTER_DEFAULT_ANGLE;
        }
        int ticks = (int) Math.round(ANGLE_TO_TICKS * (Math.toDegrees(shootingAngle) - STARTING_ANGLE));
        telemetry.addData("ticks", ticks);
        anglerLeft.setPosition(ticks);
        anglerRight.setPosition(ticks);
        if (usingIMU) {
            if (
                    Math.abs(
                            Math.abs(shooterIMU.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS))
                                    - shootingAngle) < Math.toRadians(3)) {
                telemetry.addLine(String.format(
                        "shooting angle %.2f imu angle %.2f",
                        shootingAngle,
                        Math.abs(shooterIMU.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS))
                ));
                return Step.Status.SUCCESS;
            }
            return Step.Status.RUNNING;
        } else {
            telemetry.addLine("setup angler done");
            telemetry.update();
            return Step.Status.SUCCESS;

        }
    }


    public Action shootTop(int velocity, double shootingAngle, double waitTime){
        return new SequentialAction(shootingTopTriangle(velocity,shootingAngle), new SleepAction(waitTime), shootAction());
    }

    public void shootingPID(double baseTargetTps) {

        double vL = shooterLeft.getVelocity();
        double vR = shooterRight.getVelocity();
        telemetry.addData("Left Vel",shooterLeft.getVelocity());
        telemetry.addData("Right Vel",shooterRight.getVelocity());

        // Base target with temporary recovery boost
        long now = System.currentTimeMillis();
        double targetTps = baseTargetTps;
        if (now < boostUntilMs) {
            targetTps *= (1.0 + RECOVERY_BOOST_PCT);
        }

        // Cross-coupled setpoints to keep L/R matched
        double eDiff = vL - vR; // positive if left is faster
        double leftTargetTps  = targetTps - (K_VEL_LEFT * eDiff);
        double rightTargetTps = targetTps + (K_VEL_RIGHT * eDiff);

        shooterLeft.setVelocity(leftTargetTps);
        shooterRight.setVelocity(rightTargetTps);
        telemetry.addData("Left target tps", leftTargetTps);
        telemetry.addData("Right target tps", rightTargetTps);
        telemetry.addData("Diff", eDiff);
        telemetry.update();

        // Ready-to-fire gating
        boolean inBandL = Math.abs(vL - leftTargetTps) <= tolTps;
        boolean inBandR = Math.abs(vR - rightTargetTps) <= tolTps;
        boolean matched = Math.abs(vL - vR) <= deltaTolTps;
        boolean readyBasic = inBandL && inBandR && matched;

        if (readyBasic) {
            if (readySinceMs == Long.MAX_VALUE) readySinceMs = now;
        } else {
            readySinceMs = Long.MAX_VALUE;
        }
    }
    public Action shootingTopTriangle(int velocity, double shootingAngle) {

        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {


                shooterLeft.setPower(1);
                shooterRight.setPower(1);
                shootingPID(velocity);
                anglerLeft.setPosition((int) ((shootingAngle-STARTING_ANGLE) * ANGLE_TO_TICKS));
                anglerRight.setPosition((int) ((shootingAngle-STARTING_ANGLE) * ANGLE_TO_TICKS));
                return false;
            }
        };

    }
    public Action turnToDepot(double heading){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                turnPower = 0.3;

                if (vision.checkTag() != null) {
                    x = vision.checkTag().rawPose.x;
                    if (x < heading) {
                        turnPower = -0.3;
                    } else {
                        turnPower = 0.3;
                    }
                    drive.setDrivePowers(0, 0, turnPower);
                    telemetry.addData("x", x);
                    if (Math.abs(x - heading) < 20) {
                        return false;
                    }else{
                        return true;
                    }
                }
                telemetry.addData("i am here", "realy ");
                telemetry.update();
                return false;
            }
        };
    }
    public Action shootBottom(int velocity, double shootingAngle, double waitTime){
        return new SequentialAction(shootingBottomTriangle(velocity, shootingAngle), new SleepAction(waitTime), shootAction());
    }
    public Action shootingBottomTriangle(int velocity, double shootingAngle) {

        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {


                bottomShot(velocity,shootingAngle);
                return false;
            }
        };

    }

    public void bottomShot(int velocity, double shootingAngle) {
        shooterLeft.setPower(1);
        shooterRight.setPower(1);
        shootingPID(velocity);
        anglerLeft.setPosition((int) ((shootingAngle-STARTING_ANGLE) * ANGLE_TO_TICKS));
        anglerRight.setPosition((int) ((shootingAngle-STARTING_ANGLE) * ANGLE_TO_TICKS));
        telemetry.addLine("motors powered");
        telemetry.update();
    }


    public Action shootingBottomTriangleAuto(int angle, int power) {

        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                br.setPower(1);
                bl.setPower(1);
                shooterLeft.setVelocity(power);
                shooterRight.setVelocity(power);
                anglerLeft.setPosition((int) ((angle-STARTING_ANGLE) * ANGLE_TO_TICKS));
                anglerRight.setPosition((int) ((angle-STARTING_ANGLE) * ANGLE_TO_TICKS));
                telemetry.addLine("motors powered");
                telemetry.update();
                return false;
            }
        };

    }
    public static double selectiveAverage(double value, double a, double b) {
        int count = 1;
        double sum = value;
        if (Math.abs(a - value) <= 0.1) {
            sum += a;
            count++;
        }
        if (Math.abs(b - value) <= 0.1) {
            sum += b;
            count++;
        }
        return sum / count;
    }

    public Action setupFlywheelAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                telemetry.addLine("Shooting setup started");
                return setupFlywheels() == Step.Status.RUNNING;
            }
        };
    }

    public Step setupFlywheelStepAction() {
        return new Step() {
            @Override
            public Step.Status runStep(TelemetryPacket telemetryPacket) {
                return setupFlywheels();
            }
        };
    }

    public Action setupAnglerAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return setupAngler() == Step.Status.RUNNING;
            }
        };
    }

    public Step setupAnglerStepAction() {
        return new Step() {
            @Override
            public Step.Status runStep(TelemetryPacket telemetryPacket) {
                return setupAngler();
            }
        };
    }

    public void shoot() {

        bl.setPower(1);
        br.setPower(1);
    }

    public Action shootAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                shoot();
                telemetry.addLine("Shooting");
                telemetry.update();
                return false;
            }
        };
    }

    public Step setupShootStepAction() {
        return new Step() {
            @Override
            public Step.Status runStep(TelemetryPacket telemetryPacket) {
                telemetry.addLine("shoot starts");
                shoot();
                telemetry.addLine("shoot done");
                telemetry.update();
                return Status.SUCCESS;
            }
        };
    }

    public void stop() {
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        br.setPower(0);
        bl.setPower(0);
        telemetry.addLine("Shotting stoped 1");
    }

    public void shutdown() {
        anglerRight.setPosition(0);
        anglerLeft.setPosition(0);
    }

    public Action stopAction(){

        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                stop();
                telemetry.addLine("Shooting stopped");
                return false;
            }
        };
    }

    public Step stopStepAction(){

        return new Step() {
            @Override
            public Step.Status runStep(TelemetryPacket telemetryPacket) {
                stop();
                telemetry.addLine("Shooting stopped");
                return Status.SUCCESS;
            }
        };
    }

    //    double calcSpeed() {
//        double d = (blDist.getDistance(DistanceUnit.METER) + brDist.getDistance(DistanceUnit.METER)) / 2;
//        return (d * Math.sqrt(GRAVITY / (d - VisionSystem.DELTA_Y))) / SPEED_MULTIPLIER;
//    }
//
    public double calculateShootingSpeed(double d, double shootingAngle){
        //
        double deltaH = DELTA_Y;
        if (d * Math.tan(shootingAngle) - deltaH > 0) {
            return Math.sqrt(
                    (GRAVITY * d * d) /
                            (2.0 * Math.pow(Math.cos(shootingAngle), 2) *
                                    (d * Math.tan(shootingAngle) - deltaH)));
        }  else {
            return -1.0;
        }

    }

    /**
     * Returns true if Tag 20 is visible with pose info (can shoot), false otherwise.
     * When true, also prints Tag 20 pose to telemetry. Does not print anything when false.
     */
    public boolean checkShootPoosible() {
        telemetry.addLine(":shootin check");
        AprilTagDetection tag = vision.checkTag();
        if ( tag == null ) {
            telemetry.addLine("No tag");
            return false;
        }
//        telemetry.addLine("Got April tag");
//        double targetDistance = getDistanceToTargetMeters(tag);
//        telemetry.addLine(String.format("Distance to target %.2f height %.2f",
//                targetDistance, DELTA_Y));
        return true;
    }


    public double getDistanceToTargetMeters(AprilTagDetection tag) {
        return tag.rawPose.z * 0.0254;
    }

    public double calculateShootingAngle(double targetDistance) {
        double smallerAngle = Math.atan(1-Math.sqrt(DELTA_Y/targetDistance));
//        telemetry.addData("small angle", Math.toDegrees(smallerAngle));
        double steeperAngle = Math.atan(1+Math.sqrt(DELTA_Y/targetDistance));
//        telemetry.addData("steeper angle", Math.toDegrees(steeperAngle));
        if (steeperAngle < SHOOTER_MAX_ANGLE) {
            return steeperAngle;
        }
        double middleAngle = Math.atan(2*DELTA_Y/targetDistance);
        if (middleAngle < SHOOTER_MAX_ANGLE) {
            return middleAngle;
        }
        return smallerAngle;
    }

}