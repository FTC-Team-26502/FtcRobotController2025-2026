package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

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

public class ShooterSystem {
    private static final double DEFAULT_FLYWHEEL_SPEED = 3000; // ticks per second
    private static final double SHOOTER_DEFAULT_ANGLE = Math.PI/4;
    private final Telemetry telemetry;
    private DcMotorEx anglerLeft, anglerRight;
    private DcMotorEx shooterLeft, shooterRight;
    private DistanceSensor blDist, brDist;
    private CRServo bl, br;
    private VisionSystem vision;
    protected final IMU shooterIMU;
    public boolean manualOverride = false;

    private final double ANGLER_SPEED = 0.05;
    private final double ANGLE_TO_TICKS = (1/360.0) * 537.6;
    private final double GRAVITY = 9.80665;
    public static final double DELTA_Y = 0.9;
    protected final double SHOOTER_MAX_ANGLE = Math.toRadians(55);
    protected double shootingAngle = -1.0;
    private boolean usingIMU = false;

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
        int ticks = (int) Math.round(ANGLE_TO_TICKS * Math.toDegrees(shootingAngle));
        telemetry.addData("ticks", ticks);
        anglerLeft.setPower(ANGLER_SPEED);
        anglerRight.setPower(ANGLER_SPEED);
        anglerLeft.setTargetPosition(ticks);
        anglerRight.setTargetPosition(ticks);
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
            if (Math.abs(anglerLeft.getTargetPosition() - anglerLeft.getCurrentPosition() ) < 3 ) {
                return Step.Status.SUCCESS;
            } else {
                return Step.Status.RUNNING;
            }

        }
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
        anglerLeft.setPower(0);
        anglerRight.setPower(0);
        br.setPower(0);
        bl.setPower(0);
        telemetry.addLine("Shotting stoped 1");
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

