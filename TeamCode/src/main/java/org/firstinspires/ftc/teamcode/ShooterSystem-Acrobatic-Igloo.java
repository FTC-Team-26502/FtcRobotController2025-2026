//package org.firstinspires.ftc.teamcode;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//public class ShooterSystem {
//    private DcMotorEx anglerLeft, anglerRight;
//    private DcMotorEx shooterLeft, shooterRight;
//    private DistanceSensor blDist, brDist;
//    private CRServo bl, br;
//
//    private final double ANGLER_SPEED = 0.05;
//    private final double ANGLE_TO_TICKS = (1/360.0) * 537.6;
//    private final double DELTA_Y = 0.9;
//    private final double GRAVITY = 9.80665;
//    protected final int SPEED_MULTIPLIER = 45;
//    protected final int ANGLE_OF_SHOOTER = 45;
//
//    ShooterSystem(HardwareMap hw) {
//        anglerLeft = hw.get(DcMotorEx.class, "anglerLeft");
//        anglerRight = hw.get(DcMotorEx.class, "anglerRight");
//        shooterLeft = hw.get(DcMotorEx.class, "shooterLeft");
//        shooterRight = hw.get(DcMotorEx.class, "shooterRight");
//
//        blDist = hw.get(DistanceSensor.class, "blDistance");
//        brDist = hw.get(DistanceSensor.class, "brDistance");
//
//        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        anglerLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        anglerRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        anglerLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        anglerRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//
//        anglerRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        anglerLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        bl = hw.get(CRServo.class, "inBackLeft");
//        br = hw.get(CRServo.class, "inBackRight");
//    }
//
//    public Action buildShootingAction() {
//        return new SequentialAction(
//                new ShooterSetup(),
//                new SleepAction(0.5),
//                new ShooterFeed()
//        );
//    }
//
//    double calcSpeed() {
//        double d = (blDist.getDistance(DistanceUnit.METER) + brDist.getDistance(DistanceUnit.METER)) / 2;
//        return (d * Math.sqrt(GRAVITY / (d - DELTA_Y))) / SPEED_MULTIPLIER;
//    }
//
//    public class ShooterSetup implements Action {
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet){
//            double speed = calcSpeed();
//            shooterLeft.setPower(speed);
//            shooterRight.setPower(speed);
//            int ticks = (int) Math.round(ANGLE_TO_TICKS * ANGLE_OF_SHOOTER);
//            anglerLeft.setPower(ANGLER_SPEED);
//            anglerRight.setPower(ANGLER_SPEED);
//            anglerLeft.setTargetPosition(ticks);
//            anglerRight.setTargetPosition(ticks);
//            return true;
//        }
//
//    }
//
//    public class ShooterFeed implements Action {
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            bl.setPower(1);
//            br.setPower(1);
//            return true;
//        }
//    }
//
//    public class ShooterStop implements Action{
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet){
//            shooterLeft.setPower(0);
//            shooterRight.setPower(0);
//            anglerLeft.setPower(0);
//            anglerRight.setPower(0);
//            return true;
//        }
//    }
//}
//
