package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.roadrunner.util.AxisDirection;
import org.firstinspires.ftc.teamcode.roadrunner.util.BNO055IMUUtil;

@TeleOp
public class MixedMechanumDrive extends LinearOpMode {
  @Override
  public void runOpMode() {
    Gamepad.RumbleEffect rumble_60 = new Gamepad.RumbleEffect.Builder()
            .addStep(0.5, 0.5, 200)  //  Rumble for 500 mSec
            .addStep(0.0, 0.0, 200)  //  Pause for 500 mSec
            .addStep(0.5, 0.5, 200)  //  Rumble for 500 mSec
            .addStep(0.0, 0.0, 200)  //  Pause for 500 mSec
            .addStep(0.5, 0.5, 200)  //  Rumble for 500 mSec
            .build();

    Gamepad.RumbleEffect rumble_75 = new Gamepad.RumbleEffect.Builder()
            .addStep(1, 1, 2000)  //  Rumble for 2000 mSec
            .addStep(0.0, 0.0, 200)  //  Pause for 200 mSe
            .build();

    ElapsedTime runtime = new ElapsedTime();

    double speedMod = 1;
    boolean has_rumbled_60 = false;
    boolean has_rumbled_75 = false;
    double correction = 0;

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

      if (runtime.seconds() > 60 && !has_rumbled_60) {
        gamepad1.runRumbleEffect(rumble_60);
        gamepad2.runRumbleEffect(rumble_60);
        has_rumbled_60 = true;
      }

      if (runtime.seconds() > 75 && !has_rumbled_75) {
        gamepad1.runRumbleEffect(rumble_75);
        gamepad2.runRumbleEffect(rumble_75);
        has_rumbled_75 = true;
      }

      // communication
      if (gamepad2.right_trigger != 0) {
        gamepad1.rumble(20);
      }
      if (gamepad1.y) {
        gamepad2.rumble(20);
      }

      if (gamepad1.left_bumper) {
        speedMod = 1;
      } else if (gamepad1.right_bumper) {
        speedMod = 0.75;
      } else if (gamepad1.left_trigger != 0) {
        speedMod = 0.3;
      } else if (gamepad1.right_trigger != 0) {
        speedMod = gamepad1.right_trigger;
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
        imu.initialize(parameters); // reset imu
      }

      y = gamepad1.left_stick_y; // Remember, this is reversed!
      x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
      rx = -gamepad1.right_stick_x;

      Orientation angularOri = imu.getAngularOrientation();

      // training wheels
//      if (Math.abs(Math.toDegrees(angularOri.secondAngle)) > 2) {
//        correction = Math.toDegrees(angularOri.secondAngle) / 8;
//      } else {
//        correction = 0;
//      }

      // Read inverse IMU heading, as the IMU heading is CW positive
      botHeading = -angularOri.firstAngle;

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
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftPower = (y + x + rx) / denominator;
        backLeftPower = (y - x + rx) / denominator;
        frontRightPower = (y - x - rx) / denominator;
        backRightPower = (y + x - rx) / denominator;
      }

      front_left.setPower(frontLeftPower * speedMod + correction);
      back_left.setPower(backLeftPower * speedMod + correction);
      front_right.setPower(frontRightPower * speedMod + correction);
      back_right.setPower(backRightPower * speedMod + correction);

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

      if (gamepad2.x) {
        arm_pos = 0.4;
      }

      if (arm.getPosition() < arm_pos) {
        arm.setPosition(arm.getPosition() + 0.01);
      } else if (arm.getPosition() > arm_pos) {
        arm.setPosition(arm.getPosition() - 0.01);
      }

      l = -gamepad2.left_stick_y;

      // code for holding the lift up passively
      if (l > 0) {
        lift.setPower((l * 1/1.1) + 0.1);
      } else {
        lift.setPower((l * 1/.9) + 0.1);
      }

      if (gamepad2.right_bumper) {
        claw_pos = 0.65; // close
      } else {
        claw_pos = 1;
      }
      claw.setPosition(claw_pos);

      telemetry.addData("2nd ang", Math.toDegrees(angularOri.secondAngle));

      telemetry.addData("correction", correction);

      /*
      telemetry.addData("x1", gamepad1.touchpad_finger_1_x);
      telemetry.addData("y1", gamepad1.touchpad_finger_1_y);
      telemetry.addData("x2", gamepad1.touchpad_finger_2_x);
      telemetry.addData("y2", gamepad1.touchpad_finger_2_y);
      // */
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
