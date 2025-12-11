package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Vision ShootingCheck Test", group = "Test")
public class VissionSystemTest extends BaseCodeV2 {

    private VisionSystem vision;

    @Override
    public void runOpMode()  {
        vision = new VisionSystem(hardwareMap, telemetry);

        telemetry.addLine("Vision initialized. Press START.");
        telemetry.update();

        initOpMode(false, false, true, false, true);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            boolean canShoot = vision.shootingCheck(); // prints pose if true

            if (!canShoot) {
                telemetry.addLine("CAN'T SHOOT");
                telemetry.update();
                light.setPosition(LIGHTRED);

            } else {
                light.setPosition(LIGHTGREEN);
            }

            // Show obelisk info as well
//            int[] order = vision.getObeliskOrder();
//            if (order != null) {
//
//                telemetry.addData("Obelisk Order", "[%d, %d, %d]", order[0], order[1], order[2]);
                telemetry.addData("Obelisk Tag", vision.getObeliskPattern());
//            } else {
//                telemetry.addLine("Obelisk: none");
//            }

            sleep(50);
        }

        try { vision.close(); } catch (Exception ignored) {}
    }
}