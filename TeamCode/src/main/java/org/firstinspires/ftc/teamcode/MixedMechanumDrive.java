package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.util.AxisDirection;
import org.firstinspires.ftc.teamcode.roadrunner.util.BNO055IMUUtil;

@TeleOp
public class MixedMechanumDrive extends LinearOpMode {
  @Override
  public void runOpMode() {
    Gamepad.RumbleEffect customRumbleEffect = new Gamepad.RumbleEffect.Builder()
            .addStep(1.0, 1.0, 500)  //  Rumble for 500 mSec
            .addStep(0.0, 0.0, 500)  //  Pause for 500 mSec
            .addStep(1.0, 1.0, 500)  //  Rumble for 500 mSec
            .addStep(0.0, 0.0, 500)  //  Pause for 500 mSec
            .addStep(1.0, 1.0, 500)  //  Rumble for 500 mSec
            .build();
    ElapsedTime runtime = new ElapsedTime();

    double speedMod = 1;
    boolean has_rumbled = false;

    double claw_pos = 1;
    double arm_pos = 0;
    boolean arm_prev = false;

    double frontLeftPower = 0;
    double backLeftPower = 0;
    double frontRightPower = 0;
    double backRightPower = 0;

    double dpadx = 0;
    double dpady = 0;

    double l;

    double y;
    double x;
    double rx;
    double botHeading = 0;

    DcMotor front_right = hardwareMap.dcMotor.get("rightFront");
    DcMotor back_right = hardwareMap.dcMotor.get("rightRear");
    DcMotor front_left = hardwareMap.dcMotor.get("leftFront");
    DcMotor back_left = hardwareMap.dcMotor.get("leftRear");
    
    front_right.setDirection(DcMotorSimple.Direction.REVERSE);
    back_right.setDirection(DcMotorSimple.Direction.REVERSE);

    DcMotor lift = hardwareMap.dcMotor.get("lift");
    lift.setDirection(DcMotorSimple.Direction.REVERSE);
    lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    Servo claw = hardwareMap.servo.get("claw");
    Servo arm = hardwareMap.servo.get("arm");

    // Retrieve the IMU from the hardware map
    BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    // Technically this is the default, however specifying it is clearer
    parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
    BNO055IMUUtil.remapZAxis(imu, AxisDirection.POS_Y);
    // Without this, data retrieving from the IMU throws an exception
    imu.initialize(parameters);
    
    waitForStart();
    runtime.reset();

    if (isStopRequested()) return;

    while (opModeIsActive()) {

      if (runtime.seconds() > 60 && !has_rumbled) {
        gamepad1.runRumbleEffect(customRumbleEffect);
        gamepad2.runRumbleEffect(customRumbleEffect);
        has_rumbled = true;
      }

      if (gamepad1.left_bumper) {
        speedMod = 1;
      } else if (gamepad1.left_trigger != 0) {
        speedMod = 0.3;
      } else {
        speedMod = 0.5;
      }

      dpadx = 0;
      dpady = 0;

      if (gamepad1.dpad_up) {
        dpady += 1;
      }
      if (gamepad1.dpad_down) {
        dpady -= 1;
      }
      if (gamepad1.dpad_left) {
        dpadx -= 1;
      }
      if (gamepad1.dpad_right) {
        dpadx += 1;
      }

      if (gamepad1.y) {
        imu.initialize(parameters);
      }

      y = gamepad1.left_stick_y * speedMod; // Remember, this is reversed!
      x = gamepad1.left_stick_x * 1.1 * speedMod; // Counteract imperfect strafing
      rx = gamepad1.right_stick_x;

      // Read inverse IMU heading, as the IMU heading is CW positive
      botHeading = -imu.getAngularOrientation().firstAngle;

      if (dpadx != 0 || dpady != 0) {
        //botHeading = angleFix(botHeading);

        double rotX = dpadx * Math.cos(botHeading) - dpady * Math.sin(botHeading);
        double rotY = dpadx * Math.sin(botHeading) + dpady * Math.cos(botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(dpady) + Math.abs(dpadx) + Math.abs(rx), 1);
        frontLeftPower = (rotY + rotX + rx) / denominator;
        backLeftPower = (rotY - rotX + rx) / denominator;
        frontRightPower = (rotY - rotX - rx) / denominator;
        backRightPower = (rotY + rotX - rx) / denominator;
      } else {
        frontLeftPower = (y - x - rx);
        frontRightPower = (y + x + rx);
        backLeftPower = (y + x - rx);
        backRightPower = (y - x + rx);
      }

      front_left.setPower(frontLeftPower * speedMod);
      back_left.setPower(backLeftPower * speedMod);
      front_right.setPower(frontRightPower * speedMod);
      back_right.setPower(backRightPower * speedMod);

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
      if (arm.getPosition() < arm_pos) {
        arm.setPosition(arm.getPosition() + 0.01);
      } else if (arm.getPosition() > arm_pos) {
        arm.setPosition(arm.getPosition() - 0.01);
      }

      l = -gamepad2.left_stick_y;

      if (lift.getCurrentPosition() > 4100) {
        if (l > 0) {
          l = 0;
        }
      }

      // code for holding the lift up passively
//      if (l > 0) {
//        lift.setPower((l * 1/1.1) + 0.1);
//      } else {
//        lift.setPower((l * 1/.9) + 0.1);
//      }
      lift.setPower(l);

      if (gamepad2.right_bumper) {
        claw_pos = 0.57; // close
      } else {
        claw_pos = 1;
      }
      claw.setPosition(claw_pos);

      telemetry.addData("bot angle", Math.toDegrees(botHeading));
      telemetry.addData("lift power", lift.getPower());
      telemetry.addData("lift pos", lift.getCurrentPosition());
      telemetry.addData("arm pos", arm.getPosition());
      telemetry.addData("arm_pos", arm_pos);
//      telemetry.addData("x1", gamepad1.touchpad_finger_1_x);
//      telemetry.addData("y1", gamepad1.touchpad_finger_1_y);
//      telemetry.addData("x2", gamepad1.touchpad_finger_2_x);
//      telemetry.addData("y2", gamepad1.touchpad_finger_2_y);
      telemetry.update();
    }
  }

  public double angleFix(double angle) {
    angle = Math.toDegrees(angle);

    // translate to 360
    if (angle < 0) {
      angle = 360 - angle;
    }

    // add 180
    angle += 180;

    // mod 360
    angle = angle % 360;

    // translate back to -180 to 180
    if (angle > 180) {
      angle = -(360 - angle);
    }

    return Math.toRadians(angle);
  }
}
