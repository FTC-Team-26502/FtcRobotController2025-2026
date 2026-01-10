package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class VisionSystem {

    private final Telemetry telemetry;

    // Set this to your actual camera mount relative to the robot center
    private final Position cameraPosition = new Position(
            DistanceUnit.INCH,
            0,   // X: right (+), left (-)
            0,   // Y: forward (+), back (-)
            11.5/2+1,   // Z: up (+), down (-)
            0
    );

    // Orientation of the camera relative to robot (Yaw, Pitch, Roll)
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(
            AngleUnit.DEGREES,
            0,   // Yaw
            -90, // Pitch
            0,   // Roll
            0
    );

    protected boolean blueAlliance;

    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;

    public VisionSystem(HardwareMap hw, Telemetry telemetry, boolean blueAlliance) {
        this.telemetry = telemetry;
        this.blueAlliance = blueAlliance;
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hw.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    public void reportTagIds() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        telemetry.addData("# AprilTags Detected", detections.size());
        if (detections.isEmpty()) {
            telemetry.addLine("No tags in view.");
        } else {
            for (AprilTagDetection d : detections) {
                String name = (d.metadata != null && d.metadata.name != null) ? d.metadata.name : "Unknown";
                telemetry.addData("Tag", String.format("ID %d  Name %s", d.id, name));
            }
        }
        telemetry.update();
    }

    public void updateTelemetry() {
        telemetryAprilTag();
        telemetry.update();
    }

    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.robotPose != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                        detection.robotPose.getPosition().x,
                        detection.robotPose.getPosition().y,
                        detection.robotPose.getPosition().z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)",
                        detection.center.x, detection.center.y));
            }
        }

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
    }

    // =========================
    // Obelisk helpers
    // =========================

    public enum ObeliskPattern {
        ID21_0_1_1(21, new int[]{0, 1, 1}),
        ID22_1_0_1(22, new int[]{1, 0, 1}),
        ID23_1_1_0(23, new int[]{1, 1, 0}),
        NONE(-1, null);

        public final int tagId;
        public final int[] order;

        ObeliskPattern(int tagId, int[] order) {
            this.tagId = tagId;
            this.order = order;
        }
    }

    public int getObeliskPattern() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        boolean seen21 = false, seen22 = false, seen23 = false;
        int dist = 0;
        for (AprilTagDetection d : detections) {
            if (d.id == 21) dist = 1;
            else if (d.id == 22) dist = 2;
            else if (d.id == 23) dist = 3;
            return d.id;
        }
//
//        if (seen21) return ObeliskPattern.ID21_0_1_1;
//        if (seen22) return ObeliskPattern.ID22_1_0_1;
//        if (seen23) return ObeliskPattern.ID23_1_1_0;
//        return ObeliskPattern.NONE;
        return dist;
    }

//    public int[] getObeliskOrder() {
////        ObeliskPattern p = getObeliskPattern();
//        return p.order; // null for NONE
//
//    }

    // =========================
    // Tag 20 shooting check
    // =========================

    private AprilTagDetection findDetectionById(int id) {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection d : detections) {
            telemetry.addData("tag id", d.id);
            if (d.id == id) return d;
        }
        telemetry.addLine("Tag not found");
        return null;
    }

    public AprilTagDetection checkTag() {
        int tagID = blueAlliance ? 20 : 24;
        telemetry.addData("tag ID", tagID);
        AprilTagDetection tag20 = findDetectionById(tagID);
            return tag20;

    }

}