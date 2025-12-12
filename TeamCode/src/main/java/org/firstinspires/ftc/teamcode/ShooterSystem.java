package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class ShooterSystem {
    private static final double DEFAULT_FLYEWHEEL_SPEED = ticksPerSecToMetersPerSec( 6720 ) * 0.5;
    private static final double SHOOTER_DEFAULT_ANGLE = Math.PI/4;
    private final Telemetry telemetry;
    private DcMotorEx anglerLeft, anglerRight;
    private DcMotorEx shooterLeft, shooterRight;
    private DistanceSensor blDist, brDist;
    private CRServo bl, br;
    private VisionSystem vision;
    protected final IMU shooterIMU;
    protected boolean manualOverride;

    private final double ANGLER_SPEED = 0.05;
    private final double ANGLE_TO_TICKS = (1/360.0) * 537.6;
    private final double GRAVITY = 9.80665;
    public static final double DELTA_Y = 0.9;
    protected final double SHOOTER_MAX_ANGLE = Math.toRadians(55);

    ShooterSystem(HardwareMap hw, Telemetry telemetry, VisionSystem vision, boolean manualOverride) {
        this.vision = vision;
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
        anglerLeft = hw.get(DcMotorEx.class, "anglerLeft");
        anglerRight = hw.get(DcMotorEx.class, "anglerRight");
        shooterLeft = hw.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hw.get(DcMotorEx.class, "shooterRight");

        blDist = hw.get(DistanceSensor.class, "blDistance");
        brDist = hw.get(DistanceSensor.class, "brDistance");

        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

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

    // Convenience: meters/sec -> ticks/sec (inverse)
    public static double metersPerSecToTicksPerSec(double metersPerSec) {
        double ticksPerRev = 28;
        double wheelRadiusMeters = 0.096;
        double wheelCircumference = 2.0 * Math.PI * wheelRadiusMeters;
        double revsPerSec = metersPerSec / wheelCircumference;
        return revsPerSec * ticksPerRev;
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
                new StepFromAction( new TimeOutAction( setupFlywheelAction(), 2, clock )),
                new StepFromAction(new TimeOutAction( setupAnglerAction(), 2, clock )),
                new StepFromAction( shootAction()),
                new StepFromAction( new SleepAction(2)),
                new StepFromAction( stopAction() )
        );
    }

    public boolean setupFlywheels() {
        double speed = DEFAULT_FLYEWHEEL_SPEED;
        if (!manualOverride) {
            AprilTagDetection tag = vision.checkTag();
            if (tag == null) {
                return false;
            }
            double tagDistance = getDistanceToTargetMeters(tag);
            ;
            double targetDistance = selectiveAverage(tagDistance,
                    blDist.getDistance(DistanceUnit.METER),
                    brDist.getDistance(DistanceUnit.METER));
            double shootingAngle = calculateShootingAngle(targetDistance);
            telemetry.addLine(String.format("target shooting angle %.2f", Math.toDegrees(shootingAngle)));
            speed = calculateShootingSpeed(targetDistance, shootingAngle); // mps
        }
        telemetry.addLine(String.format("target speed %.2f", speed));
        if (speed < 0) {
            return false;
        }
        double angularSpeed = metersPerSecToTicksPerSec(speed);
        telemetry.addLine(String.format("left target speed %.2f, left current speed %.2f",
                angularSpeed, shooterLeft.getVelocity()));
        telemetry.addLine(String.format("left target speed %.2f, left current speed %.2f",
                angularSpeed, shooterLeft.getVelocity()));
        if (Math.abs(angularSpeed - shooterLeft.getVelocity()) < 0.3 &&
                Math.abs(angularSpeed - shooterRight.getVelocity()) < 0.3) {
            return true;
        }
        shooterLeft.setVelocity(angularSpeed);
        shooterRight.setVelocity(angularSpeed);
        return false;
    }

    public boolean setupAngler() {
        double shootingAngle = SHOOTER_DEFAULT_ANGLE;
        if (!manualOverride) {
            AprilTagDetection tag = vision.checkTag();
            if (tag == null) {
                return false;
            }
            double tagDistance = getDistanceToTargetMeters(tag);
            ;
            double targetDistance = selectiveAverage(tagDistance,
                    blDist.getDistance(DistanceUnit.METER),
                    brDist.getDistance(DistanceUnit.METER));
            shootingAngle = calculateShootingAngle(targetDistance);
        }
        int ticks = (int) Math.round(ANGLE_TO_TICKS * shootingAngle);
        anglerLeft.setPower(ANGLER_SPEED);
        anglerRight.setPower(ANGLER_SPEED);
        anglerLeft.setTargetPosition(ticks);
        anglerRight.setTargetPosition(ticks);
        if (
                Math.abs(
                        Math.abs(shooterIMU.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS))
                                - shootingAngle) < Math.toRadians(3) ) {
            telemetry.addLine(String.format(
                    "shooting angle %.2f imu angle %.2f",
                    shootingAngle,
                    Math.abs(shooterIMU.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS))

            ));
            return true;
        }
        return false;
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
                telemetry.update();
                return setupFlywheels();
            }
        };
    }

    public Action setupAnglerAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
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

    public void stop() {
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        anglerLeft.setPower(0);
        anglerRight.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }

    public Action stopAction(){

        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                stop();
                telemetry.addLine("Shooting stopped");
                telemetry.update();
                return false;
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
            return Math.sqrt((GRAVITY * d * d) /
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
        AprilTagDetection tag = vision.checkTag();
        if ( tag == null ) {
            return false;
        }
        telemetry.addLine("Got April tag");
        double targetDistance = getDistanceToTargetMeters(tag);
        telemetry.addLine(String.format("Distance to target %.2f height %.2f",
                targetDistance, DELTA_Y));
        return targetDistance >= DELTA_Y;
    }


    public double getDistanceToTargetMeters(AprilTagDetection tag) {
        return tag.rawPose.z * 0.0254;
    }

    public double calculateShootingAngle(double targetDistance) {
        double smallerAngle = Math.atan(1-Math.sqrt(DELTA_Y/targetDistance));
        double steeperAngle = Math.atan(1-Math.sqrt(2*DELTA_Y/targetDistance));
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

