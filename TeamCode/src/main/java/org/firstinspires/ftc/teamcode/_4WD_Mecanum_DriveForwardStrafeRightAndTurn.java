package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "_4WD_Mecanum_DriveForwardStrafeRightAndTurn (Blocks to Java)")
public class _4WD_Mecanum_DriveForwardStrafeRightAndTurn extends LinearOpMode {

  private DcMotor left_rear;
  private DcMotor left_front;
  private DcMotor right_rear;
  private DcMotor right_front;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    left_rear = hardwareMap.get(DcMotor.class, "left_rear");
    left_front = hardwareMap.get(DcMotor.class, "left_front");
    right_rear = hardwareMap.get(DcMotor.class, "right_rear");
    right_front = hardwareMap.get(DcMotor.class, "right_front");

    // Put initialization blocks here.
    right_rear.setDirection(DcMotorSimple.Direction.REVERSE);
    left_front.setDirection(DcMotorSimple.Direction.REVERSE);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      right_rear.setPower(0.5);
      right_front.setPower(0.5);
      left_rear.setPower(0.5);
      left_front.setPower(0.5);
      sleep(10000);
      right_rear.setPower(0);
      right_front.setPower(0);
      left_rear.setPower(0);
      left_front.setPower(0);
      sleep(5000);
      right_rear.setPower(0.5);
      right_front.setPower(-0.5);
      left_rear.setPower(-0.5);
      left_front.setPower(0.5);
      sleep(3000);
      right_rear.setPower(0);
      right_front.setPower(0);
      left_rear.setPower(0);
      left_front.setPower(0);
      sleep(5000);
      right_rear.setPower(0.5);
      right_front.setPower(0.5);
      left_rear.setPower(-0.5);
      left_front.setPower(-0.5);
      sleep(3000);
      right_rear.setPower(0);
      right_front.setPower(0);
      left_rear.setPower(0);
      left_front.setPower(0);
      while (opModeIsActive()) {
        // Put loop blocks here.
        telemetry.update();
      }
    }
  }
}
