package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MecanumDrive extends LinearOpMode {

//  DcMotor front_right;
//  DcMotor back_right;
//  DcMotor front_left;
//  DcMotor back_left;
//  DcMotor lift;
//  Servo claw;
//  Servo arm;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    double horizontal;
    double vertical;
    double turn;
    double speedMod = 1;

    double reverse = 1;

    double claw_pos = 1;
    double arm_pos = 0;
    boolean arm_prev = false;

    DcMotor front_right = hardwareMap.dcMotor.get("rightFront");
    DcMotor back_right = hardwareMap.dcMotor.get("rightRear");
    DcMotor front_left = hardwareMap.dcMotor.get("leftFront");
    DcMotor back_left = hardwareMap.dcMotor.get("leftRear");
    DcMotor lift = hardwareMap.dcMotor.get("lift");

    Servo claw = hardwareMap.servo.get("claw");
    Servo arm = hardwareMap.servo.get("arm");
    
    front_right.setDirection(DcMotorSimple.Direction.REVERSE);
    back_right.setDirection(DcMotorSimple.Direction.REVERSE);

    lift.setDirection(DcMotorSimple.Direction.REVERSE);
    lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    waitForStart();
    
    if (opModeIsActive()) {
      while (opModeIsActive()) {

        if (gamepad1.right_trigger != 0) {
          reverse = -1;
        } else {
          reverse = 1;
        }

        if (gamepad2.y && !arm_prev) {
          if (arm_pos == 1) {
            arm_pos = 0;
          } else {
            arm_pos = 1;
          }
          arm_prev = true;
        } else if (!gamepad2.y) {
          arm_prev = false;
        }
        arm.setPosition(arm_pos);
        
        // Assign movement controls
        vertical = gamepad1.left_stick_y * reverse;
        horizontal = -gamepad1.left_stick_x * reverse;
        turn = gamepad1.right_stick_x;

        front_left.setPower((-turn + vertical + horizontal) * speedMod);
        front_right.setPower((turn + vertical + -horizontal) * speedMod);
        back_left.setPower((-turn + vertical + -horizontal) * speedMod);
        back_right.setPower((turn + vertical + horizontal) * speedMod);

        if (gamepad1.left_bumper) {
          speedMod = 1;
        } else if (gamepad1.left_trigger != 0) {
          speedMod = 0.3;
        } else {
          speedMod = 0.5;
        }

//        if (gamepad2.left_stick_y != 0) {
//          lift.setPower(-gamepad2.left_stick_y);
//        } else {
//          lift.setPower(0.1);
//        }
        lift.setPower(-gamepad2.left_stick_y);

        if (gamepad2.right_bumper) {
          claw_pos = 0.45; // close
        } else {
          claw_pos = 1;
        }
        claw.setPosition(claw_pos);

//        if (gamepad1.dpad_up) {
//          lift.setPower(1);
//        } else if (gamepad1.dpad_down) {
//          lift.setPower(-1);
//        } else {
//          lift.setPower(0.1);
//        }

        telemetry.addData("lift_pos", lift.getCurrentPosition());
        telemetry.update();
      }
    }
  }
}
